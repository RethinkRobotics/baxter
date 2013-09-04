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
        @param gripper - robot limb <left/right> on which the gripper is mounted

        Interface class for a gripper on the Baxter Research Robot.
        """
        self.name = gripper + '_gripper'

        ns = '/robot/end_effector/' + self.name + "/"

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

        self.set_parameters(defaults=True)

    def _on_gripper_state(self, state):
        self._state = deepcopy(state)

    def _on_gripper_prop(self, properties):
        self._prop = deepcopy(properties)

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)

    def command(self, cmd, block=False, test=lambda: True, time=0.0,
                args=None, msg=''):
        """
        @param cmd (string)   - string of known gripper commands
        @param block (bool)   - command is blocking or non-blocking [False]
        @param test (func)   - test function for command validation
        @param time (float)   - timeout in seconds for command evaluation
        @param args dict({str:float}) - dictionary of parameter:value
        @param msg (string)   - error message string for failed command

        Set the parameters that will describe the position command execution.
        Percentage of maximum (0-100) for each parameter
        """
        self._cmd_msg.id = self.hardware_id()
        self._cmd_msg.command = cmd
        self._cmd_msg.args = ''
        if args != None:
            self._cmd_msg.args = JSONEncoder().encode(args)
        self._pub_cmd.publish(self._cmd_msg)
        if block:
            dataflow.wait_for(
                test=test,
                timeout=time,
                timeout_msg=msg,
                body=lambda: self._pub_cmd.publish(self._cmd_msg)
            )

    def valid_parameters_text(self):
        """
        Text describing valid gripper parameters.
        """
        return """Valid gripper parameters are
        PARAMETERS:
        velocity      - Velocity at which a position move will execute
        moving_force  - Force threshold at which a move will stop
        holding_force - Force at which a grasp will continue holding
        dead_zone     - Position dead band within move considered successful
        ALL PARAMETERS (0-100)
        """

    def valid_parameters(self):
        """
        Returns dict of available gripper parameters with default parameters.
        """
        valid = {'velocity':50.0,
                 'moving_force':40.0,
                 'holding_force':30.0,
                 'dead_zone':5.0
                 }
        return valid

    def set_parameters(self, params=None, defaults=False):
        """
        @param params dict({str:float}) - dictionary of parameter:value

        Set the parameters that will describe the position command execution.
        Percentage of maximum (0-100) for each parameter
        """
        valid_params = self.valid_parameters()
        if defaults:
            for param in valid_params.keys():
                self._params[param] = valid_params[param]
        if params is None:
            params = {}
        for key in params.keys():
            if key in valid_params.keys():
                self._params[key] = params[key]
            else:
                msg = ("Invalid parameter: %s provided. %s" %
                       (key, valid_parameters_text(),))
                rospy.logwarn(msg)
        cmd = EndEffectorCommand.CMD_CONFIGURE
        self.command(cmd, args=self._params)

    def reset(self, timeout=2.0, block=True):
        """
        @param timeout (float)   - timeout in seconds for reset success
        @param block (bool) - command is blocking or non-blocking [False]

        Resets the gripper state removing any errors.
        """
        cmd = EndEffectorCommand.CMD_RESET
        error_msg = ("Unable to successfully reset the %s" % (self.name,))
        self.command(
            cmd,
            block,
            test=lambda: self._state.error == False,
            time=timeout,
            msg=error_msg
        )

    def reboot(self, timeout=2.0, block=True):
        """
        @param timeout (float)   - timeout in seconds for reboot success
        @param block (bool) - command is blocking or non-blocking [False]

        Power cycle the gripper removing calibration information and any errors.
        """
        cmd = EndEffectorCommand.CMD_REBOOT
        error_msg = ("Unable to successfully reboot the %s" % (self.name,))
        self.command(
            cmd,
            block,
            test=lambda: (self._state.calibrated == False and
                     self._state.error == False),
            time=timeout,
            msg=error_msg
        )
        rospy.sleep(0.5) # Allow extra time for reboot to complete
        self.set_parameters(defaults=True)

    def calibrate(self, timeout=5.0, block=True):
        """
        @param timeout (float)   - timeout in seconds for calibration success
        @param block (bool) - command is blocking or non-blocking [False]

        Calibrate the gripper setting maximum and minimum travel distance.
        """
        cmd = EndEffectorCommand.CMD_CALIBRATE
        error_msg = ("Unable to successfully calibrate the %s" % (self.name,))
        self.command(
            cmd,
            block,
            test=lambda: (self._state.calibrated == True),
            time=timeout,
            msg=error_msg
        )
        self.set_parameters(defaults=True)


    def stop(self, timeout=5.0, block=True):
        """
        @param timeout (float)   - timeout in seconds for stop success
        @param block (bool) - command is blocking or non-blocking [False]

        Stop the gripper at the current position and apply holding force.
        """
        cmd = EndEffectorCommand.CMD_STOP
        error_msg = ("Unable to verify the %s has stopped" % (self.name,))
        self.command(
            cmd,
            block,
            test=lambda: self._state.moving == False,
            time=timeout,
            msg=error_msg
        )

    def command_position(self, position, block=False, timeout=5.0):
        """
        @param position (float) - in % 0=close 100=open

        Command the gripper position movement.
        from minimum (0) to maximum (100)
        """
        if not self._state.calibrated:
            msg = ("Unable to command %s positions until calibrated" %
                   self.name)
            raise IOError(errno.EPERM, msg)
            rospy.logwarn(msg)
            return

        cmd = EndEffectorCommand.CMD_GO
        arguments = {"position": self._clip(position)}
        error_msg = ("Unable to verify the %s position move" % (self.name,))
        self.command(
            cmd,
            block,
            test=lambda: (fabs(self._state.position - position) <
                          self._params['dead_zone'] or self._state.gripping),
            time=timeout,
            args=arguments,
            msg=error_msg
        )

    def set_velocity(self, velocity):
        """
        @param velocity (float) - in % 0=stop 100=max [50.0]

        Set the velocity at which the gripper position movement will execute.
        """
        velocity_param = dict(velocity=self._clip(velocity))
        self.set_parameters(params=velocity_param, defaults=False)

    def set_moving_force(self, force):
        """
        @param force (float) - in % 0=none 100=max [30.0]

        Set the moving force threshold at which the position move will execute.
        When exceeded, the gripper will stop trying to achieve the commanded
        position.
        """
        moving = dict(moving_force=self._clip(force))
        self.set_parameters(params=moving, defaults=False)

    def set_holding_force(self, force):
        """
        @param force (float) - in % 0=none 100=max [30.0]

        Set the force at which the gripper will continue applying after a
        position command has completed either from successfully achieving the
        commanded position, or by exceeding the moving force threshold.
        """
        holding = dict(holding_force=self._clip(force))
        self.set_parameters(params=holding, defaults=False)

    def set_dead_band(self, dead_band):
        """
        @param dead_band (float) - in % of full position [5.0]

        Set the gripper dead band describing the position error threshold
        where a move will be considered successful.
        """
        dead_band_param = dict(dead_band=self._clip(dead_band))
        self.set_parameters(params=dead_band_param, defaults=False)

    def open(self, block=False):
        """
        @param block (bool) - open command is blocking or non-blocking [False]

        Commands maximum gripper position.
        """
        self.command_position(100.0, block)

    def close(self, block=False):
        """
        @param block (bool) - close command is blocking or non-blocking [False]

        Commands minimum gripper position.
        """
        self.command_position(0.0, block)

    def parameters(self):
        """
        Returns dict of parameters describing the position command execution.
        """
        return deepcopy(self._params)

    def calibrated(self):
        """
        Returns bool describing gripper calibration state.
        (0:Not Calibrated, 1:Calibrated)
        """
        return self._state.calibrated is True

    def ready(self):
        """
        Returns bool describing if the gripper ready, i.e. is calibrated, not
        busy (as in resetting or rebooting), and not moving.
        """
        return self._state.ready is True

    def moving(self):
        """
        Returns bool describing if the gripper is in motion
        """
        return self._state.moving is True

    def gripping(self):
        """
        Returns bool describing if the position move has been preempted by a
        position command exceeding the moving_force threshold denoting a grasp.
        """
        return self._state.gripping is True

    def missed(self):
        """
        Returns bool describing if the position move has completed without
        exceeding the moving_force threshold denoting a grasp
        """
        return self._state.missed is True

    def error(self):
        """
        Returns bool describing if the gripper is in an error state.
        Error states can be caused by over/undervoltage, over/under current,
        motor faults, etc.
        Errors can be cleared with a gripper reset/reboot. If persistent please
        contact Rethink Robotics for further debugging.
        """
        return self._state.error is True

    def position(self):
        """
        Returns the current gripper position as a ratio (0-100) of the total
        gripper travel.
        """
        return deepcopy(self._state.position)

    def force(self):
        """
        Returns the current measured gripper force as a ratio (0-100) of the
        total force applicable.
        """
        return deepcopy(self._state.force)

    def has_force(self):
        """
        Returns bool describing if the gripper is capable of force control.
        """
        return self._prop.controls_force is True

    def has_position(self):
        """
        Returns bool describing if the gripper is capable of position control.
        """
        return self._prop.controls_position is True

    def type(self):
        """
        Returns string describing the gripper type.
        Known types are 'suction', 'electric', and 'custom'.
        An unknown or no gripper attached to the research robot will be
        reported as 'custom'.
        """
        return {
        EndEffectorProperties.SUCTION_CUP_GRIPPER: 'suction',
        EndEffectorProperties.ELECTRIC_GRIPPER: 'electric',
        EndEffectorProperties.CUSTOM_GRIPPER: 'custom',
        }.get(self._prop.ui_type, None)

    def hardware_id(self):
        """
        Returns unique hardware id number. This is required for sending commands
        to the gripper.
        """
        return deepcopy(self._state.id)

    def hardware_version(self):
        """
        Returns string describing the gripper hardware.
        """
        return deepcopy(self._prop.product)

    def firmware_version(self):
        """
        Returns the current gripper firmware revision.
        """
        return deepcopy(self._prop.firmware_rev)
