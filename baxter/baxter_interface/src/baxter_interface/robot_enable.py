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
import os
import sys
import time

import roslib
roslib.load_manifest('baxter_interface')
import rospy
import std_msgs.msg
import baxter_msgs.msg

class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

    enable()    - enable all joints
    disable()   - disable all joints
    reset()     - reset all joints, reset all jrcp faults, disable the robot
    stop()      - stop the robot, similar to hitting the e-stop button

    """
    def __init__(self):
        self._state = None
        self._state_sub = rospy.Subscriber(
            '/sdk/robot/state',
            baxter_msgs.msg.AssemblyState,
            self._state_callback)

        rate = rospy.Rate(10)
        timeout = 10
        waited = 0.0
        while self._state == None and waited < timeout:
            rate.sleep()

        if self._state == None:
            raise IOError(errno.ETIMEDOUT, "Failed to get current robot state from /sdk/robot/state")

    def _state_callback(self, msg):
        self._state = msg

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
            self.reset()
        self._toggle_enabled(True)

    def disable(self):
        """
        Disable all joints
        """
        self._toggle_enabled(False)

    def _loop(self, test, publisher, *args):
        """
        Publish *args on publisher at 10 Hz until test passes or two seconds
        has elapsed.

        @param test     - function passed the current robot state, should return
                          True if the state matches what is desired.
        @parm publisher - Publisher to publish *args on.
        @param *args    - Passed to publisher.
        """
        rate = rospy.Rate(10)
        timeout = 2
        waited = 0.0

        while waited < timeout:
            if test(self._state):
                return True

            publisher.publish(*args)
            rate.sleep()
            waited += 0.10

        return False

    def _toggle_enabled(self, status):
        test = lambda state: state.enabled == status
        if test(self._state):
            return

        pub = rospy.Publisher('/robot/set_super_enable', std_msgs.msg.Bool)

        if not self._loop(test, pub, status):
            raise IOError(errno.ETIMEDOUT, "Failed to %sable robot" % ('en' if status else 'dis',))

    def reset(self):
        """
        Reset all joints.  Trigger JRCP hardware to reset all faults.  Disable
        the robot.
        """
        test = lambda state: state.enabled == False\
                and state.stopped == False \
                and state.error == False \
                and state.estop_button == 0 \
                and state.estop_source == 0

        if test(self._state):
            return

        pub = rospy.Publisher('/robot/set_super_reset', std_msgs.msg.Empty)

        if not self._loop(test, pub):
            raise IOError(errno.ETIMEDOUT, "Failed to reset robot")

    def stop(self):
        """
        Simulate an e-stop button being pressed.  Robot must be reset to clear
        the stopped state.
        """
        test = lambda state: state.stopped == True

        pub = rospy.Publisher('/robot/set_super_stop', std_msgs.msg.Empty)
        if not self._loop(test, pub):
            raise IOError(errno.ETIMEDOUT, "Failed to stop robot")

