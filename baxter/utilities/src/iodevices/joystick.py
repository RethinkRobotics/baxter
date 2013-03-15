#!/usr/bin/env python

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

import os.path
import sys
import roslib
import rospy
from errno import EINVAL

from sensor_msgs.msg import Joy

class Transition(object):
    """ local base class to monitor transitions
    The transition is measured when read
    """
    def __init__(self, joystick, name):
        self._joystick = joystick
        self._name = name
    def changed(self):
        raise NotImplementedError()

class ButtonTransition(Transition):
    """ monitor button transitions
    The transition is measured when read
    """
    def __init__(self, joystick, name, down_val=1, up_val=0):
        super(Joystick.ButtonTransition, self).__init__(joystick, name)
        self._value = up_val
        self._down_val = down_val
        self._up_val = up_val

    def _changed(self):
        new_value = self._joystick.get_value(self._name, self._value)
        equal = (new_value == self._value)
        self._value = new_value
        return not equal

    def down(self):
        if (self._changed() and (self._value == self._down_val)):
            return True
        return False

    def up(self):
        return (self._changed() and (self._value == self._up_val))

class StickTransition(Transition):
    """ monitor transitions in stick movement
    The transition is measured when read
    """
    def __init__(self, joystick, name, epsilon=0.001):
        super(Joystick.StickTransition, self).__init__(joystick, name)
        self._epsilon = epsilon
        self._value = self._get_value()

    def _get_value(self):
        self.joystick.get_value(self._name, self._value)

    def get_value(self):
        if self._changed():
            self._value = self._get_value()
        return self._value

    def _changed(self):
        return abs(self._get_value() - self._value) > self.epsilon

class Joystick(object):
    """ Abstract base class to handle joystick input
    """

    def __init__(self, scale=0.5, offset= 0.5, deadband=0.01):
        """ Maps joystick input to robot control
        Sets up the bindings
        Args:
            pad_type(str): the type of controller used ('xbox' or 'logitech')
            scale(float): scaling applied to joystick values.
                                        raw joystick valuess are in [1.0...-1.0]
            offset(float): offset applied to joystick values, post-scaling
            deadband(float): deadband applied to scaled, offset values

        """
        self.sub = rospy.Subscriber("/joy", Joy, self.on_joy)
        self.scale = scale
        self.offset = offset
        self.deadband = deadband
        self.controls = {}
        self.new_data = False

    def _stick_value(value, deadband, scale, offset):
        """ Local function to condition raw joystick values
        Args:
            value(float): the raw joystick axis value
            deadband(float): deadband applied to the raw value
            scale(float): scaling applied to raw value 
            offset(float): offset applied to the scaled value

        Returns:
            the deadbanded value of the axis
        """
        return (value * scale) + offset if (value > deadband or value < -deadband) else 0

    def on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """
        raise NotImplementedError()

    def get_value(self, control, default=0):
        if control in self.controls:
            return self.controls[control]
        return default

    def create_button_changed_dict(self, *buttonNames):
        """ Creates a dictionary holding the transition
        objects  for each button
        """
        button_dict = {}
        for name in buttonNames:
            button_dict[name] = Joystick.ButtonTransition(self, name)
        return button_dict

    def create_stick_changed_dict(self, *stickNames):
        """ Creates a dictionary holding the transition
        objects  for each joystick
        """
        stick_dict = {}
        for name in stickNames:
            stick_dict[name] = self.StickTransition(self, name)
        return stick_dict

class XBoxController(Joystick):
    """ Xbox specialization of Joystick
    """
    def __init__(self, scale=0.5, offset= 0.5, deadband=0.01):
        super(Joystick, self).__init__(scale, offset, deadband)

    def on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """

        self.controls['btnLeft'] = (msg.buttons[2] == 1)
        self.controls['btnUp'] = (msg.buttons[3] == 1)
        self.controls['btnDown'] = (msg.buttons[0] == 1)
        self.controls['btnRight'] = (msg.buttons[1] == 1)

        self.controls['dPadUp'] = (msg.axes[7] > 0.5)
        self.controls['dPadDown'] = (msg.axes[7] < -0.5)
        self.controls['dPadLeft'] = (msg.axes[6] > 0.5)
        self.controls['dPadRight'] = (msg.axes[6] < -0.5)

        self.controls['leftStickHorz']  = _stick_value(msg.axes[0], self.deadband, self.scale, self.offset)
        self.controls['leftStickVert']  = _stick_value(msg.axes[1], self.deadband, self.scale, self.offset)
        self.controls['rightStickHorz'] = _stick_value(msg.axes[3], self.deadband, self.scale, self.offset)
        self.controls['rightStickVert'] = _stick_value(msg.axes[4], self.deadband, self.scale, self.offset)

        self.controls['leftBumper'] = (msg.buttons[4] == 1)
        self.controls['rightBumper'] = (msg.buttons[5] == 1)
        self.controls['leftTrigger'] = (msg.axes[2] > 0.5)
        self.controls['rightTrigger'] = (msg.axes[5] > 0.5)

        self.controls['function1'] = (msg.buttons[6] == 1)
        self.controls['function2'] = (msg.buttons[10] == 1)
        self.new_data = True

class LogitechController(Joystick):
    """ Logitech specialization of Joystick
    """
    def __init__(self, scale=0.5, offset= 0.5, deadband=0.01):
        super(Joystick, self).__init__(scale, offset, deadband)

    def on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """

        self.controls['btnLeft'] = (msg.buttons[0] == 1)
        self.controls['btnUp'] = (msg.buttons[3] == 1)
        self.controls['btnDown'] = (msg.buttons[1] == 1)
        self.controls['btnRight'] = (msg.buttons[2] == 1)

        self.controls['dPadUp'] = (msg.axes[5] > 0.5)
        self.controls['dPadDown'] = (msg.axes[5] < -0.5)
        self.controls['dPadLeft'] = (msg.axes[4] > 0.5)
        self.controls['dPadRight'] = (msg.axes[4] < -0.5)

        self.controls['leftStickHorz']  = _stick_value(msg.axes[0], self.deadband, self.scale, self.offset)
        self.controls['leftStickVert']  = _stick_value(msg.axes[1], self.deadband, self.scale, self.offset)
        self.controls['rightStickHorz'] = _stick_value(msg.axes[2], self.deadband, self.scale, self.offset)
        self.controls['rightStickVert'] = _stick_value(msg.axes[3], self.deadband, self.scale, self.offset)

        self.controls['leftBumper'] = (msg.buttons[4] == 1)
        self.controls['rightBumper'] = (msg.buttons[5] == 1)
        self.controls['leftTrigger'] = (msg.buttons[6] == 1)
        self.controls['rightTrigger'] = (msg.buttons[7] == 1)

        self.controls['function1'] = (msg.buttons[8] == 1)
        self.controls['function2'] = (msg.buttons[9] == 1)
        self.new_data = True
