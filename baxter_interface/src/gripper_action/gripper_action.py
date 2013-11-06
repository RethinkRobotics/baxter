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

"""
Baxter RSDK Gripper Action Server
"""
from math import fabs

import rospy

import actionlib

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandFeedback,
    GripperCommandResult,
)

import baxter_interface


class GripperActionServer(object):
    def __init__(self, gripper, reconfig_server):
        self._dyn = reconfig_server
        self._ee = gripper + '_gripper'
        self._ns = 'robot/end_effector/' + self._ee + '/gripper_action'
        self._gripper = baxter_interface.Gripper(gripper)
        # Store Gripper Type
        self._type = self._gripper.type()
        if self._type == 'custom':
            msg = ("Stopping %s action server - %s gripper not capable of "
                   "gripper actions" % (self._gripper.name, self._type))
            rospy.logerr(msg)
            return

        # Verify Grippers Have No Errors and are Calibrated
        if self._gripper.error():
            self._gripper.reboot()
            if self._gripper.error():
                msg = ("Stopping %s action server - Unable to clear error" %
                       self._gripper.name)
                rospy.logerr(msg)
                return
        if not self._gripper.calibrated():
            self._gripper.calibrate()
            if not self._gripper.calibrated():
                msg = ("Stopping %s action server - Unable to calibrate" %
                       self._gripper.name)
                rospy.logerr(msg)
                return

        # Action Server
        self._server = actionlib.SimpleActionServer(
            self._ns,
            GripperCommandAction,
            execute_cb=self._on_gripper_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()

        # Action Feedback/Result
        self._fdbk = GripperCommandFeedback()
        self._result = GripperCommandResult()

        # Initialize Parameters
        self._prm = self._gripper.parameters()
        self._timeout = 5.0

    def _get_gripper_parameters(self):
        self._timeout = self._dyn.config[self._ee + '_timeout']
        if self._type == 'electric':
            self._prm['dead_zone'] = self._dyn.config[self._ee + '_goal']
            self._prm['velocity'] = self._dyn.config[self._ee + '_velocity']
            self._prm['moving_force'] = self._dyn.config[self._ee +
                                                         '_moving_force']
            self._prm['holding_force'] = self._dyn.config[self._ee +
                                                          '_holding_force']
        elif self._type == 'suction':
            self._prm['vacuum_sensor_threshold'] = self._dyn.config[
                self._ee + '_vacuum_threshold']
            self._prm['blow_off_seconds'] = self._dyn.config[
                self._ee + '_blow_off_seconds']
        self._gripper.set_parameters(parameters=self._prm)

    def _update_feedback(self, position):
        if self._type == 'electric':
            self._fdbk.position = self._gripper.position()
            self._fdbk.effort = self._gripper.force()
            self._fdbk.stalled = (self._gripper.force() >
                                  self._gripper.parameters()['moving_force'])
            self._fdbk.reached_goal = (fabs(self._gripper.position() -
                                            position) <
                                       self._gripper.parameters()['dead_zone'])
        if self._type == 'suction':
            self._fdbk.effort = self._gripper.vacuum_sensor()
            if position > 50.0:
                self._fdbk.reached_goal = (not self._gripper.sucking() and
                                           not self._gripper.blowing())
            else:
                self._fdbk.reached_goal = self._gripper.gripping()
        self._result = self._fdbk
        self._server.publish_feedback(self._fdbk)

    def _command_gripper(self, position):
        if self._type == 'electric':
            self._gripper.command_position(position, block=False)
        elif self._type == 'suction':
            if position > 50.0:
                self._gripper.open(block=False)
            else:
                # if infinite timeout, command suction for 1 hour
                if self._timeout < 0.0:
                    self._timeout = 3600.0
                self._gripper.close(block=False, timeout=self._timeout)

    def _check_state(self, position):
        if self._type == 'electric':
            return (self._gripper.force() >
                    self._gripper.parameters()['moving_force'] or
                    fabs(self._gripper.position() - position) <
                    self._gripper.parameters()['dead_zone'])
        elif self._type == 'suction':
            if position > 50.0:
                return (not self._gripper.sucking() and
                        not self._gripper.blowing())
            else:
                return self._gripper.gripping()

    def _on_gripper_action(self, goal):
        # Store position and effort from call
        # Position to 0:100 == close:open
        position = goal.command.position
        effort = goal.command.max_effort
        # Apply max effort if specified < 0
        if effort == -1.0:
            effort = 100.0

        # Check for errors
        if self._gripper.error():
            rospy.logerr("%s: Gripper error - please restart action server." %
                         (self._action_name,))
            self._server.set_aborted()

        # Pull parameters that will define the gripper actuation
        self._get_gripper_parameters()

        # Reset feedback/result
        self._update_feedback(position)

        # 20 Hz gripper state rate
        control_rate = rospy.Rate(20.0)

        # Record start time
        start_time = rospy.get_time()

        # Set the moving_force/vacuum_threshold based on max_effort provided
        # If effort not specified (0.0) use parameter server value
        if self._type == 'electric':
            if fabs(effort) < 0.0001:
                effort = self._prm['moving_force']
            self._gripper.set_moving_force(effort)
        elif self._type == 'suction':
            if fabs(effort) < 0.0001:
                effort = self._prm['vacuum_sensor_threshold']
            self._gripper.set_vacuum_threshold(effort)

        def now_from_start(start):
            return rospy.get_time() - start

        # Continue commanding goal until success or timeout
        while ((now_from_start(start_time) < self._timeout or
               self._timeout < 0.0) and not rospy.is_shutdown()):
            if self._server.is_preempt_requested():
                self._gripper.stop()
                rospy.loginfo("%s: Gripper Action Preempted" %
                              (self._action_name,))
                self._server.set_preempted(self._result)
                return
            self._update_feedback(position)
            if self._check_state(position):
                self._server.set_succeeded(self._result)
                return
            self._command_gripper(position)
            control_rate.sleep()

        # Gripper failed to achieve goal before timeout/shutdown
        self._gripper.stop()
        if not rospy.is_shutdown():
            rospy.logerr("%s: Gripper Command Not Achieved in Allotted Time" %
                         (self._action_name,))
        self._update_feedback(position)
        self._server.set_aborted(self._result)
