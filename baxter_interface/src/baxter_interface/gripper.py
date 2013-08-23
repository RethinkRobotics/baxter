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

from copy import deepcopy
from math import fabs

from json import (
    JSONDecoder,
    JSONEncoder,
)

import roslib
roslib.load_manifest('baxter_interface')
import rospy

from baxter_core_msgs.msg import (
    EndEffectorCommand,
    EndEffectorProperties,
    EndEffectorState,
)

import dataflow

class Gripper(object):
    def __init__(self, gripper):
        """
        Interface class for a gripper on the Baxter robot.

        @param gripper - gripper to interface
        """
        self.name = gripper

        ns = '/robot/end_effector/' + self.name + '_gripper/'

        self._state = None
        self._prop = EndEffectorProperties()
        self._cmd_msg = EndEffectorCommand()

        self._params = {}

        self._pub_cmd = rospy.Publisher(ns + 'command', EndEffectorCommand)

        self._state_sub = rospy.Subscriber(
                              ns + 'state',
                              EndEffectorState,
                              self._on_gripper_state
                          )

        self._prop_sub = rospy.Subscriber(
                                   ns + 'properties',
                                   EndEffectorProperties,
                                   self._on_gripper_prop
                               )

        dataflow.wait_for(
            lambda: not self._state is None,
            timeout=5.0,
            timeout_msg=("Failed to get current state from %s" % 
                         (ns + 'state',))
        )

        self.configure(defaults=True)

    def _on_gripper_state(self, state):
        self._state = deepcopy(state)

    def _on_gripper_prop(self, properties):
        self._prop = deepcopy(properties)

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)

    def _command(self, cmd, block=False, test=True, time=0.0, args='', msg=''):
        self._cmd_msg.id = self.hardware_id()
        self._cmd_msg.command = cmd
        if len(args) != 0:
            self._cmd_msg.args = JSONEncoder().encode(args)
        self._pub_cmd.publish(self._cmd_msg)
        if block:
            dataflow.wait_for(
                test=lambda: test(),
                timeout=time,
                timeout_msg=msg,
                body=lambda: self._pub_cmd.publish(self._cmd_msg)
            )

    def configure(self, defaults=False):
        if defaults:
            self._params['position'] = 100.0
            self._params['velocity'] = 50.0
            self._params['moving_force'] = 30.0
            self._params['holding_force'] = 30.0
            self._params['dead_zone'] = 5.0
        cmd = EndEffectorCommand.CMD_CONFIGURE
        self._command(cmd, args=self._params)

    def reset(self, timeout=2.0, block=True):
        """
        Reset the gripper
        """
        cmd = EndEffectorCommand.CMD_RESET
        error_msg = ("Unable to successfully reset the %s gripper" % 
                     (self.name,))
        self._command(cmd, block, lambda: self._state.error == False,
                      time=timeout, msg=error_msg)

    def reboot(self, timeout=2.0, block=True):
        """
        Reboot the gripper
        """
        cmd = EndEffectorCommand.CMD_REBOOT
        error_msg = ("Unable to successfully reboot the %s gripper" % 
                     (self.name,))
        self._command(cmd, block, lambda: self._state.calibrated == False,
                      time=timeout, msg=error_msg)
        self.configure(defaults=True)

    def calibrate(self, timeout=5.0, block=True):
        """
        Calibrate the gripper
        """
        cmd = EndEffectorCommand.CMD_CALIBRATE
        error_msg = ("Unable to successfully calibrate the %s gripper" % 
                     (self.name,))
        self._command(cmd, block, lambda: self._state.calibrated == True,
                      time=timeout, msg=error_msg)
        self.configure(defaults=True)


    def stop(self, block=True):
        """
        Stop the gripper at the current position and force
        """
        cmd = EndEffectorCommand.CMD_STOP
        error_msg = ("Unable to verify the %s gripper has stopped" % 
                     (self.name,))
        self._command(cmd, block, lambda: self._state.moving == False,
                      time=timeout, msg=error_msg)

    def command_position(self, position, block=False, timeout=5.0):
        """
        Set the gripper position

        @param position (float) - in % 0=close 100=open
        """
        cmd = EndEffectorCommand.CMD_GO
        arguments = {"position": self._clip(position)}
        error_msg = ("Unable to verify the %s gripper position move" % 
                     (self.name,))
        self._command(cmd, block,
                      lambda: (fabs(self._state.position - position) < 
                               self._params['dead_zone'] or
                               self._state.gripping),
                      time=timeout, args=arguments, msg=error_msg)

    def set_velocity(self, velocity):
        """
        Set the gripper velocity

        @param velocity (float) - in % 0=stop 100=max
        """
        self._params['velocity'] = self._clip(velocity)
        self.configure(defaults=False)

    def set_moving_force(self, force):
        """
        Set the gripper force

        @param force (float) - in % 0=none 100=max
        """
        self._params['force'] = self._clip(force)
        self.configure(defaults=False)

    def set_holding_force(self, force):
        """
        Set the gripper holding force

        @param force (float) - in % 0=none 100=max
        """
        self._params['force'] = self._clip(force)
        self.configure(defaults=False)

    def set_dead_band(self, dead_band):
        """
        Set the gripper dead band

        @param dead_band (float) - in % of full position
        """
        self._params['dead_band'] = self._clip(dead_band)
        self.configure(defaults=False)

    def inc_position_command(self, position, block=False):
        """
        Increment the gripper position

        @param position (float) - percentage to increment by
        """
        self.command_position = self._clip(self._state.position + position)

    def inc_velocity(self, velocity):
        """
        Increment the gripper velocity

        @param velocity (float) - percentage to increment by
        """
        self._params['velocity'] = self._clip(self._params['velocity'] +
                                              velocity)
        self.configure(defaults=False)

    def inc_moving_force(self, force):
        """
        Increment the gripper force

        @param force (float) - percentage to increment by
        """
        self._params['moving_force'] = self._clip(self._params['moving_force'] +
                                                  force)

    def inc_holding_force(self, force):
        """
        Increment the gripper holding force

        @param force (float) - percentage to increment by
        """
        self._params['holding_force'] = self._clip(self._params['holding_force']
                                                   + force)
        self.configure(defaults=False)

    def inc_dead_band(self, dead_band):
        """
        Increment the gripper dead band

        @param dead_band (float) - percentage to increment by
        """
        self._params['dead_band'] = self._clip(self._params['dead_band']
                                                   + dead_band)
        self.configure(defaults=False)

    def open(self, block=False):
        self.command_position(100.0, block)

    def close(self, block=False):
        self.command_position(0.0, block)

    def enabled(self):
        return self._state.enabled is True

    def calibrated(self):
        return self._state.calibrated is True

    def ready(self):
        return self._state.ready is True

    def moving(self):
        return self._state.moving is True

    def gripping(self):
        return self._state.gripping is True

    def missed(self):
        return self._state.missed is True

    def error(self):
        return self._state.error is True

    def is_reverse(self):
        return self._state.reversed is True

    def position(self):
        return deepcopy(self._state.position)

    def force(self):
        return deepcopy(self._state.force)

    def has_force(self):
        return self._prop.hasForce is True

    def has_position(self):
        return self._prop.hasPosition is True

    def type(self):
        if self._prop.ui_type == EndEffectorProperties.NO_GRIPPER:
            return 'no_gripper'
        elif self._prop.ui_type == EndEffectorProperties.SUCTION_CUP_GRIPPER:
            return 'suction'
        elif self._prop.ui_type == EndEffectorProperties.ELECTRIC_GRIPPER:
            return 'electric'
        elif self._prop.ui_type == EndEffectorProperties.CUSTOM_GRIPPER:
            return 'custom'
        else:
            return None

    def hardware_id(self):
        return deepcopy(self._state.id)

    def version(self):
        return deepcopy(self._prop.firmware_rev)
