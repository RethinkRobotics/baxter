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

import rospy

import baxter_dataflow

from baxter_core_msgs.msg import (
    EndEffectorCommand,
    EndEffectorProperties,
    EndEffectorState,
)


class Gripper(object):
    def __init__(self, gripper):
        """
        @param gripper - robot limb <left/right> on which the gripper
                         is mounted

        Interface class for a gripper on the Baxter Research Robot.
        """
        self.name = gripper + '_gripper'

        ns = '/robot/end_effector/' + self.name + "/"

        self._state = None
        self._prop = EndEffectorProperties()

        self._parameters = dict()

        self._cmd_pub = rospy.Publisher(ns + 'command', EndEffectorCommand)

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

        # Wait for the gripper state message to be populated
        baxter_dataflow.wait_for(
                          lambda: not self._state is None,
                          timeout=5.0,
                          timeout_msg=("Failed to get state from %s" %
                                       (ns + 'state',))
                          )

        # Wait for the gripper type to be populated
        baxter_dataflow.wait_for(
                          lambda: not self.type() is None,
                          timeout=5.0,
                          timeout_msg=("Failed to get properties from %s" %
                                       (ns + 'properties',))
                          )
        self.set_parameters(defaults=True)

    def _on_gripper_state(self, state):
        self._state = deepcopy(state)

    def _on_gripper_prop(self, properties):
        self._prop = deepcopy(properties)

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)

    def _capablity_warning(self, cmd):
        msg = ("%s %s - not capable of '%s' command" %
               (self.name, self.type(), cmd))
        rospy.logwarn(msg)

    def command(self, cmd, block=False, test=lambda: True,
                time=0.0, args=None):
        """
        @param cmd (string) - string of known gripper commands
        @param block (bool) - command is blocking or non-blocking [False]
        @param test (func) - test function for command validation
        @param time (float) - timeout in seconds for command evaluation
        @param args (dict({str:float})) - dictionary of parameter:value

        Set the parameters that will describe the position command execution.
        Percentage of maximum (0-100) for each parameter
        """
        ee_cmd = EndEffectorCommand()
        ee_cmd.id = self.hardware_id()
        ee_cmd.command = cmd
        ee_cmd.args = ''
        if args != None:
            ee_cmd.args = JSONEncoder().encode(args)
        self._cmd_pub.publish(ee_cmd)
        if block:
            return baxter_dataflow.wait_for(
                       test=test, timeout=time,
                       raise_on_error=False,
                       body=lambda: self._cmd_pub.publish(ee_cmd),
                   )
        else:
            return True

    def valid_parameters_text(self):
        """
        Text describing valid gripper parameters.
        """
        if self.type() == 'electric':
            return """Valid gripper parameters for the electric gripper are
            PARAMETERS:
            velocity - Velocity at which a position move will execute
            moving_force - Force threshold at which a move will stop
            holding_force - Force at which a grasp will continue holding
            dead_zone - Position deadband within move considered successful
            ALL PARAMETERS (0-100)
            """
        elif self.type() == 'suction':
            return """Valid gripper parameters for the suction gripper are
            PARAMETERS:
            vacuum_sensor_threshold - Measured suction threshold denoting grasp
            blow_off_seconds - Time in seconds to blow air on release
            ALL PARAMETERS (0-100)
            """
        else:
            return "No valid parameters for %s %s." % (self.type(), self.name)

    def valid_parameters(self):
        """
        Returns dict of available gripper parameters with default parameters.
        """
        valid = dict()
        if self.type() == 'electric':
            valid = dict({'velocity': 50.0,
                         'moving_force': 40.0,
                         'holding_force': 30.0,
                         'dead_zone': 5.0,
                         })
        elif self.type() == 'suction':
            valid = dict({'vacuum_sensor_threshold': 18.0,
                          'blow_off_seconds': 0.4,
                          })
        return valid

    def set_parameters(self, parameters=None, defaults=False):
        """
        @param parameters dict({str:float}) - dictionary of parameter:value

        Set the parameters that will describe the position command execution.
        Percentage of maximum (0-100) for each parameter
        """
        valid_parameters = self.valid_parameters()
        if defaults:
            self._parameters = valid_parameters
        if parameters is None:
            parameters = dict()
        for key in parameters.keys():
            if key in valid_parameters.keys():
                self._parameters[key] = parameters[key]
            else:
                msg = ("Invalid parameter: %s provided. %s" %
                       (key, self.valid_parameters_text(),))
                rospy.logwarn(msg)
        cmd = EndEffectorCommand.CMD_CONFIGURE
        self.command(cmd, args=self._parameters)

    def reset(self, timeout=2.0, block=True):
        """
        @param timeout (float) - timeout in seconds for reset success
        @param block (bool) - command is blocking or non-blocking [False]

        Resets the gripper state removing any errors.
        """
        if self.type() != 'electric':
            return self._capablity_warning('reset')

        cmd = EndEffectorCommand.CMD_RESET
        return self.command(
                            cmd,
                            block,
                            test=lambda: self._state.error == False,
                            time=timeout,
                            )

    def reboot(self, timeout=2.0, block=True):
        """
        @param timeout (float) - timeout in seconds for reboot success
        @param block (bool) - command is blocking or non-blocking [False]

        Power cycle the gripper removing calibration information and any
        errors.
        """
        if self.type() != 'electric':
            return self._capablity_warning('reboot')

        cmd = EndEffectorCommand.CMD_REBOOT
        self.command(
                     cmd,
                     block,
                     test=lambda: (self._state.enabled == True and
                                   self._state.error != True and
                                   self._state.ready == True),
                     time=timeout,
                     )
        rospy.sleep(0.5)  # Allow extra time for reboot to complete
        self.set_parameters(defaults=True)

    def calibrate(self, timeout=5.0, block=True):
        """
        @param timeout (float) - timeout in seconds for calibration success
        @param block (bool) - command is blocking or non-blocking [False]

        Calibrate the gripper setting maximum and minimum travel distance.
        """
        if self.type() != 'electric':
            return self._capablity_warning('calibrate')

        cmd = EndEffectorCommand.CMD_CALIBRATE
        self.command(
                     cmd,
                     block,
                     test=lambda: (self._state.calibrated == True),
                     time=timeout,
                     )
        self.set_parameters(defaults=True)

    def stop(self, timeout=5.0, block=True):
        """
        @param timeout (float) - timeout in seconds for stop success
        @param block (bool) - command is blocking or non-blocking [False]

        Stop the gripper at the current position and apply holding force.
        """
        if self.type() == 'custom':
            return self._capablity_warning('stop')

        if self.type() == 'electric':
            cmd = EndEffectorCommand.CMD_STOP
            stop_test = lambda: self._state.moving == False
        elif self.type() == 'suction':
            timeout = max(self._parameters['blow_off_seconds'], timeout)
            cmd = EndEffectorCommand.CMD_RELEASE
            stop_test = lambda: (not self.sucking() and not self.blowing())
        return self.command(
                            cmd,
                            block,
                            test=stop_test,
                            time=timeout,
                            )

    def command_position(self, position, block=False, timeout=5.0):
        """
        @param position (float) - in % 0=close 100=open

        Command the gripper position movement.
        from minimum/closed (0) to maximum/open (100)
        """
        if self.type() != 'electric':
            return self._capablity_warning('command_position')

        if not self._state.calibrated:
            msg = "Unable to command %s position until calibrated" % self.name
            rospy.logwarn(msg)

        cmd = EndEffectorCommand.CMD_GO
        arguments = {"position": self._clip(position)}
        return self.command(
                            cmd,
                            block,
                            test=lambda: (fabs(self._state.position - position)
                                          < self._parameters['dead_zone']
                                          or self._state.gripping),
                            time=timeout,
                            args=arguments,
                            )

    def command_suction(self, block=False, timeout=5.0):
        """
        Command the gripper suction.
        Timeout describes how long the suction will be applied while trying
        to determine a grasp (vacuum threshold exceeded) has been achieved.
        """
        if self.type() != 'suction':
            return self._capablity_warning('command_suction')

        cmd = EndEffectorCommand.CMD_GO
        arguments = {"grip_attempt_seconds": timeout}
        return self.command(
                            cmd,
                            block,
                            test=self.vacuum,
                            time=timeout,
                            args=arguments,
                            )

    def set_velocity(self, velocity):
        """
        @param velocity (float) - in % 0=stop 100=max [50.0]

        Set the velocity at which the gripper position movement will execute.
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_velocity')

        velocity_param = dict(velocity=self._clip(velocity))
        self.set_parameters(parameters=velocity_param, defaults=False)

    def set_moving_force(self, force):
        """
        @param force (float) - in % 0=none 100=max [30.0]

        Set the moving force threshold at which the position move will execute.
        When exceeded, the gripper will stop trying to achieve the commanded
        position.
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_moving_force')

        moving = dict(moving_force=self._clip(force))
        self.set_parameters(parameters=moving, defaults=False)

    def set_holding_force(self, force):
        """
        @param force (float) - in % 0=none 100=max [30.0]

        Set the force at which the gripper will continue applying after a
        position command has completed either from successfully achieving the
        commanded position, or by exceeding the moving force threshold.
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_holding_force')

        holding = dict(holding_force=self._clip(force))
        self.set_parameters(parameters=holding, defaults=False)

    def set_dead_band(self, dead_band):
        """
        @param dead_band (float) - in % of full position [5.0]

        Set the gripper dead band describing the position error threshold
        where a move will be considered successful.
        """
        if self.type() != 'electric':
            return self._capablity_warning('set_dead_band')

        dead_band_param = dict(dead_zone=self._clip(dead_band))
        self.set_parameters(parameters=dead_band_param, defaults=False)

    def set_vacuum_threshold(self, threshold):
        """
        @param threshold (float) - in % of measured vacuum range [18.0]

        Set the gripper suction threshold describing the threshold at which the
        measured suction (vacuum achieved) must exceed to denote a successful
        grasp.
        """
        if self.type() != 'suction':
            return self._capablity_warning('set_vacuum_threshold')

        threshold_param = dict(vacuum_sensor_threshold=self._clip(threshold))
        self.set_parameters(parameters=threshold_param, defaults=False)

    def set_blow_off(self, blow_off):
        """
        @param blow_off (float) - Time in seconds to blow air on release [0.4]

        Sets the blow_off parameter. This parameter will be used on a stop
        command with the suction gripper, ceasing suction and blowing air
        from the suction gripper for the seconds specified by this method.

        Note: This blow off will only be commanded after the previous suction
        command returned a successful grasp (suction threshold was exceeded)
        """
        if self.type() != 'suction':
            return self._capablity_warning('set_blow_off')

        blow_off_param = dict(blow_off_seconds=blow_off)
        self.set_parameters(parameters=blow_off_param, defaults=False)

    def open(self, block=False, timeout=5.0):
        """
        @param block (bool) - open command is blocking or non-blocking [False]
        @param timeout (float) - timeout in seconds for open command success

        Commands maximum gripper position.
        """
        if self.type() == 'custom':
            return self._capablity_warning('open')
        elif self.type() == 'electric':
            return self.command_position(position=100.0, block=block,
                                         timeout=timeout)
        elif self.type() == 'suction':
            return self.stop(block=block, timeout=timeout)

    def close(self, block=False, timeout=5.0):
        """
        @param block (bool) - close command is blocking or non-blocking [False]
        @param timeout (float) - timeout in seconds for close command success

        Commands minimum gripper position.
        """
        if self.type() == 'custom':
            return self._capablity_warning('close')
        elif self.type() == 'electric':
            return self.command_position(position=0.0, block=block,
                                         timeout=timeout)
        elif self.type() == 'suction':
            return self.command_suction(block=block, timeout=timeout)

    def parameters(self):
        """
        Returns dict of parameters describing the gripper command execution.
        """
        return deepcopy(self._parameters)

    def calibrated(self):
        """
        Returns bool describing gripper calibration state.
        (0:Not Calibrated, 1:Calibrated)
        """
        return self._state.calibrated == True

    def ready(self):
        """
        Returns bool describing if the gripper ready, i.e. is calibrated, not
        busy (as in resetting or rebooting), and not moving.
        """
        return self._state.ready == True

    def moving(self):
        """
        Returns bool describing if the gripper is in motion
        """
        return self._state.moving == True

    def gripping(self):
        """
        Returns bool describing if the position move has been preempted by a
        position command exceeding the moving_force threshold denoting a grasp.
        """
        return self._state.gripping == True

    def missed(self):
        """
        Returns bool describing if the position move has completed without
        exceeding the moving_force threshold denoting a grasp
        """
        return self._state.missed == True

    def error(self):
        """
        Returns bool describing if the gripper is in an error state.

        Error states can be caused by over/undervoltage, over/under current,
        motor faults, etc.

        Errors can be cleared with a gripper reset/reboot. If persistent please
        contact Rethink Robotics for further debugging.
        """
        return self._state.error == True

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

    def vacuum_sensor(self):
        """
        Returns the value (0-100) of the current vacuum sensor reading as a
        percentage of the full vacuum sensor range.

        The message field contains an 8-bit integer representation of the
        vacuum sensor, this function converts that integer to the percentage of
        the full sensor range.
        """
        if self.type() != 'suction':
            return self._capablity_warning('vacuum_sensor')
        sensor = JSONDecoder().decode(self._state.state)['vacuum sensor']
        return (sensor / 255.0) * 100.0

    def vacuum(self):
        """
        Returns bool describing if the vacuum sensor threshold has been
        exceeded during a command_suction event.
        """
        if self.type() != 'suction':
            return self._capablity_warning('vacuum')
        return JSONDecoder().decode(self._state.state)['vacuum']

    def blowing(self):
        """
        Returns bool describing if the gripper is currently blowing.
        """
        if self.type() != 'suction':
            return self._capablity_warning('blowing')
        return JSONDecoder().decode(self._state.state)['blowing']

    def sucking(self):
        """
        Returns bool describing if the gripper is currently sucking.
        """
        if self.type() != 'suction':
            return self._capablity_warning('sucking')
        return JSONDecoder().decode(self._state.state)['sucking']

    def has_force(self):
        """
        Returns bool describing if the gripper is capable of force control.
        """
        return self._prop.controls_force == True

    def has_position(self):
        """
        Returns bool describing if the gripper is capable of position control.
        """
        return self._prop.controls_position == True

    def type(self):
        """
        Returns string describing the gripper type.

        Known types are 'suction', 'electric', and 'custom'. An unknown or no
        gripper attached to the research robot will be reported as 'custom'.
        """
        return {
        EndEffectorProperties.SUCTION_CUP_GRIPPER: 'suction',
        EndEffectorProperties.ELECTRIC_GRIPPER: 'electric',
        EndEffectorProperties.CUSTOM_GRIPPER: 'custom',
        }.get(self._prop.ui_type, None)

    def hardware_id(self):
        """
        Returns unique hardware id number. This is required for sending
        commands to the gripper.
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
