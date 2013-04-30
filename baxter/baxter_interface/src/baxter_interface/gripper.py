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

import sys
import time
import copy

import roslib
roslib.load_manifest('baxter_interface')
import rospy

import baxter_msgs.msg as baxmsg
import std_msgs.msg as stdmsg

import dataflow

class Gripper(object):
    def __init__(self, gripper):
        """
        Interface class for a gripper on the Baxter robot.

        @param gripper - gripper to interface
        """
        self.name = gripper

        ns = '/robot/limb/' + self.name + '/accessory/gripper/'
        sdkns = '/sdk' + ns

        self._pub_enable = rospy.Publisher(
            ns + 'set_enabled',
            stdmsg.Bool)

        self._pub_reset = rospy.Publisher(
            ns + 'command_reset',
            stdmsg.Bool)

        self._pub_calibrate = rospy.Publisher(
            ns + 'command_calibrate',
            stdmsg.Empty)

        self._pub_command = rospy.Publisher(
            sdkns + 'command_set',
            baxmsg.GripperCommand)

        self._sub_identity = rospy.Subscriber(
            sdkns + 'identity',
            baxmsg.GripperIdentity,
            self._on_gripper_identity)

        self._sub_properties = rospy.Subscriber(
            sdkns + 'properties',
            baxmsg.GripperProperties,
            self._on_gripper_properties)

        self._sub_state = rospy.Subscriber(
            sdkns + 'state',
            baxmsg.GripperState,
            self._on_gripper_state)

        self._command = baxmsg.GripperCommand(
            position=0.0,
            force=30.0,
            velocity=100.0,
            holding=0.0,
            deadZone=3.0)

        self._identity = baxmsg.GripperIdentity()
        self._properties = baxmsg.GripperProperties()
        self._state = None

        dataflow.wait_for(
            lambda: not self._state is None,
            timeout=5.0,
            timeout_msg="Failed to get current gripper state from %s" % (sdkns + 'state'),
        )

    def _on_gripper_state(self, state):
        self._state = copy.deepcopy(state)

    def _on_gripper_identity(self, identity):
        self._identity = copy.deepcopy(identity)

    def _on_gripper_properties(self, properties):
        self._properties = copy.deepcopy(properties)

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)

    def enable(self, timeout=2.0):
        """
        Enable the gripper
        """
        self._pub_enable.publish(True)
        dataflow.wait_for(
            test=lambda: self.enabled(),
            timeout=timeout,
            body=lambda: self._pub_enable.publish(True)
            )

    def disable(self, timeout=2.0):
        """
        Disable the gripper
        """
        self._pub_enable.publish(False)
        dataflow.wait_for(
            test=lambda: not self.enabled(),
            timeout=timeout,
            body=lambda: self._pub_enable.publish(False)
            )

    def reset(self, timeout=2.0):
        """
        Reset the gripper
        """
        self._pub_reset.publish(False)
        dataflow.wait_for(
            test=lambda: not self.error(),
            timeout=timeout,
            body=lambda: self._pub_reset.publish(False)
            )

    def reboot(self, timeout=2.0):
        """
        Reboot the gripper
        """
        self._pub_reset.publish(True)
        dataflow.wait_for(
            test=lambda: not self.calibrated(),
            timeout=timeout,
            body=lambda: self._pub_reset.publish(True)
            )

    def calibrate(self, timeout=5.0):
        """
        Calibrate the gripper
        """
        self._pub_calibrate.publish(stdmsg.Empty())
        self.enable()
        dataflow.wait_for(
            test=lambda: self.calibrated(),
            timeout=timeout,
            body=lambda: self._pub_calibrate.publish(stdmsg.Empty())
            )

    def stop(self):
        """
        Stop the gripper at the current position and force
        """
        cmd = baxmsg.GripperCommand()
        cmd.position = self._state.position
        cmd.velocity = 0.0
        cmd.force = self._state.force
        cmd.holding = self._state.force
        cmd.deadZone = self._command.deadZone
        self._pub_command.publish(cmd)

    def set_position(self, position):
        """
        Set the gripper position

        @param position (float) - in % 0=close 100=open
        """
        self._command.position = self._clip(position)
        self._pub_command.publish(self._command)

    def set_velocity(self, velocity):
        """
        Set the gripper velocity

        @param velocity (float) - in % 0=stop 100=max
        """
        self._command.velocity = self._clip(velocity)
        self._pub_command.publish(self._command)

    def set_force(self, force):
        """
        Set the gripper force

        @param force (float) - in % 0=none 100=max
        """
        self._command.force = self._clip(force)
        self._pub_command.publish(self._command)

    def set_holding_force(self, force):
        """
        Set the gripper holding force

        @param force (float) - in % 0=none 100=max
        """
        self._command.holding = self._clip(force)
        self._pub_command.publish(self._command)

    def set_dead_band(self, dead_band):
        """
        Set the gripper dead band

        @param dead_band (float) - in % of full position
        """
        self._command.deadZone = self._clip(dead_band)
        self._pub_command.publish(self._command)

    def inc_position(self, position):
        """
        Increment the gripper position

        @param position (float) - percentage to increment by
        """
        self._command.position = self._clip(position + self._command.position)
        self._pub_command.publish(self._command)

    def inc_velocity(self, velocity):
        """
        Increment the gripper velocity

        @param velocity (float) - percentage to increment by
        """
        self._command.velocity = self._clip(velocity + self._command.velocity)
        self._pub_command.publish(self._command)

    def inc_force(self, force):
        """
        Increment the gripper force

        @param force (float) - percentage to increment by
        """
        self._command.force = self._clip(force + self._command.force)
        self._pub_command.publish(self._command)

    def inc_holding_force(self, force):
        """
        Increment the gripper holding force

        @param force (float) - percentage to increment by
        """
        self._command.holding = self._clip(force + self._command.holding)
        self._pub_command.publish(self._command)

    def inc_dead_band(self, dead_band):
        """
        Increment the gripper dead band

        @param dead_band (float) - percentage to increment by
        """
        self._command.deadZone = self._clip(dead_band + self._command.deadZone)
        self._pub_command.publish(self._command)

    def open(self):
        self.set_position(100.0)

    def close(self):
        self.set_position(0.0)

    def enabled(self):
        return self._state.enabled == baxmsg.GripperState.STATE_TRUE

    def calibrated(self):
        return self._state.calibrated == baxmsg.GripperState.STATE_TRUE

    def ready(self):
        return self._state.ready == baxmsg.GripperState.STATE_TRUE

    def moving(self):
        return self._state.moving == baxmsg.GripperState.STATE_TRUE

    def gripping(self):
        return self._state.gripping == baxmsg.GripperState.STATE_TRUE

    def missed(self):
        return self._state.missed == baxmsg.GripperState.STATE_TRUE

    def error(self):
        return self._state.error == baxmsg.GripperState.STATE_TRUE

    def position(self):
        return self._state.position

    def force(self):
        return self._state.force

    def has_force(self):
        return self._properties.hasForce

    def has_position(self):
        return self._properties.hasPosition

    def is_reverse(self):
        return self._properties.isReverse

    def name(self):
        return self._identity.name

    def type(self):
        if self._identity.type == baxmsg.GripperIdentity.SUCTION_CUP_GRIPPER:
            return 'suction'
        elif self._identity.type == baxmsg.GripperIdentity.PNEUMATIC_GRIPPER:
            return 'pneumatic'
        elif self._identity.type == baxmsg.GripperIdentity.ELECTRIC_GRIPPER:
            return 'electric'
        else:
            return None

    def hardware_id(self):
        return self._identity.hardware_id

    def version(self):
        return "%d.%d.%d" % (
            self._identity.version_major,
            self._identity.version_minor,
            self._identity.revision_lsb)

