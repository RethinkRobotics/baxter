# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import errno

import rospy

from std_msgs.msg import (
    Bool,
    Empty,
)

import baxter_dataflow

from baxter_core_msgs.msg import (
    AssemblyState,
)


class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

    enable()  - enable all joints
    disable() - disable all joints
    reset()   - reset all joints, reset all jrcp faults, disable the robot
    stop()    - stop the robot, similar to hitting the e-stop button
    """
    def __init__(self):
        self._state = None
        state_topic = 'robot/state'
        self._state_sub = rospy.Subscriber(state_topic,
                                           AssemblyState,
                                           self._state_callback
                                           )

        baxter_dataflow.wait_for(
            lambda: not self._state is None,
            timeout=2.0,
            timeout_msg=("Failed to get robot state on %s" %
            (state_topic,)),
        )

    def _state_callback(self, msg):
        self._state = msg

    def _toggle_enabled(self, status):

        pub = rospy.Publisher('robot/set_super_enable', Bool)

        baxter_dataflow.wait_for(
            test=lambda: self._state.enabled == status,
            timeout=2.0 if status else 5.0,
            timeout_msg=("Failed to %sable robot" %
                         ('en' if status else 'dis',)),
            body=lambda: pub.publish(status),
        )
        rospy.loginfo("Robot %s", ('Enabled' if status else 'Disabled'))

    def state(self):
        """
        Returns the last known robot state.
        """
        return self._state

    def enable(self):
        """
        Enable all joints
        """
        if self._state.stopped:
            if self._state.estop_button == AssemblyState.ESTOP_BUTTON_PRESSED:
                rospy.logfatal("E-STOP is ASSERTED; Disengage and Reset robot")
                raise IOError(errno.EREMOTEIO,
                              "Failed to Enable: E-Stop Engaged")
            else:
                rospy.loginfo("Robot Stopped: Attempting Reset...")
                self.reset()
        self._toggle_enabled(True)

    def disable(self):
        """
        Disable all joints
        """
        self._toggle_enabled(False)

    def reset(self):
        """
        Reset all joints.  Trigger JRCP hardware to reset all faults.  Disable
        the robot.
        """
        is_reset = lambda: (self._state.enabled == False and
                            self._state.stopped == False and
                            self._state.error == False and
                            self._state.estop_button == 0 and
                            self._state.estop_source == 0
                            )

        pub = rospy.Publisher('robot/set_super_reset', Empty)

        rospy.loginfo("Resetting robot...")
        error_msg = """Failed to reset robot.
Please verify that the ROS_IP or ROS_HOSTNAME environment variables are set and
resolvable. For more information please visit:
https://github.com/RethinkRobotics/sdk-docs/wiki/Rsdk-shell#initialize
"""
        baxter_dataflow.wait_for(
            test=is_reset,
            timeout=3.0,
            timeout_msg=error_msg,
            body=pub.publish,
        )

    def stop(self):
        """
        Simulate an e-stop button being pressed.  Robot must be reset to clear
        the stopped state.
        """
        pub = rospy.Publisher('robot/set_super_stop', Empty)
        baxter_dataflow.wait_for(
            test=lambda: self._state.stopped == True,
            timeout=3.0,
            timeout_msg="Failed to stop the robot",
            body=pub.publish,
        )
