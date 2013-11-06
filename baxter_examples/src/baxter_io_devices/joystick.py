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

import rospy

from sensor_msgs.msg import Joy


class ButtonTransition(object):
    """
    Monitor button transitions.

    The transition is measured when read.
    """
    def __init__(self, val_func, down_val=True, up_val=False):
        self._raw_value = val_func
        self._down_val = down_val
        self._up_val = up_val
        self._up_checked = True
        self._down_checked = False

    def down(self):
        val = self._raw_value()
        if val == self._down_val:
            if not self._down_checked:
                self._down_checked = True
                return True
        else:
            self._down_checked = False
        return False

    def up(self):
        val = self._raw_value()
        if val == self._up_val:
            if not self._up_checked:
                self._up_checked = True
                return True
        else:
            self._up_checked = False
        return False


class StickTransition(object):
    """
    Monitor transitions in stick movement.

    The transition is measured when read.
    """
    def __init__(self, val_func, epsilon=0.05):
        self._raw_value = val_func
        self._epsilon = epsilon
        self._value = 0.0

    def value(self):
        self.changed()
        return self._value

    def changed(self):
        value = self._raw_value()
        if abs(value - self._value) > self._epsilon:
            self._value = value
            return True
        return False

    def increased(self):
        value = self._raw_value()
        if (value - self._value) > self._epsilon:
            self._value = value
            return True
        return False

    def decreased(self):
        value = self._raw_value()
        if (self._value - value) > self._epsilon:
            self._value = value
            return True
        return False


class Joystick(object):
    """
    Abstract base class to handle joystick input.
    """

    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        """
        @param scale (float) - scaling applied to joystick values [1.0]
        @param offset (float) - joystick offset values, post-scaling [0.0]
        @param deadband (float) - deadband post scaling and offset [0.1]

        Maps joystick input to robot control.

        Raw joystick valuess are in [1.0...-1.0].
        """
        sub = rospy.Subscriber("/joy", Joy, self._on_joy)
        self._scale = scale
        self._offset = offset
        self._deadband = deadband
        self._controls = {}

        self._buttons = {}
        self._sticks = {}
        button_names = (
            'btnLeft', 'btnUp', 'btnDown', 'btnRight',
            'dPadUp', 'dPadDown', 'dPadLeft', 'dPadRight',
            'leftBumper', 'rightBumper',
            'leftTrigger', 'rightTrigger',
            'function1', 'function2')
        stick_names = (
            'leftStickHorz', 'leftStickVert',
            'rightStickHorz', 'rightStickVert')

        #doing this with lambda won't work
        def gen_val_func(name, type_name):
            def val_func():
                return type_name(
                    name in self._controls and self._controls[name])
            return val_func

        for name in button_names:
            self._buttons[name] = ButtonTransition(gen_val_func(name, bool))
        for name in stick_names:
            self._sticks[name] = StickTransition(gen_val_func(name, float))

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """
        raise NotImplementedError()

    def button_up(self, name):
        return self._buttons[name].up()

    def button_down(self, name):
        return self._buttons[name].down()

    def stick_changed(self, name):
        return self._sticks[name].changed()

    def stick_inc(self, name):
        return self._sticks[name].increased()

    def stick_dec(self, name):
        return self._sticks[name].decreased()

    def stick_value(self, name):
        """
        Returns:
            the deadbanded, scaled and offset value of the axis
        """
        value = self._sticks[name].value()
        if value > self._deadband or value < -self._deadband:
            return (value * self._scale) + self._offset
        return 0


class XboxController(Joystick):
    """
    Xbox specialization of Joystick.
    """
    def __init__(self, scale=1.0, offset=0.0, deadband=0.5):
        super(XboxController, self).__init__(scale, offset, deadband)

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """

        self._controls['btnLeft'] = (msg.buttons[2] == 1)
        self._controls['btnUp'] = (msg.buttons[3] == 1)
        self._controls['btnDown'] = (msg.buttons[0] == 1)
        self._controls['btnRight'] = (msg.buttons[1] == 1)

        self._controls['dPadUp'] = (msg.axes[7] > 0.5)
        self._controls['dPadDown'] = (msg.axes[7] < -0.5)
        self._controls['dPadLeft'] = (msg.axes[6] > 0.5)
        self._controls['dPadRight'] = (msg.axes[6] < -0.5)

        self._controls['leftStickHorz'] = msg.axes[0]
        self._controls['leftStickVert'] = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[3]
        self._controls['rightStickVert'] = msg.axes[4]

        self._controls['leftBumper'] = (msg.buttons[4] == 1)
        self._controls['rightBumper'] = (msg.buttons[5] == 1)
        self._controls['leftTrigger'] = (msg.axes[2] < 0.0)
        self._controls['rightTrigger'] = (msg.axes[5] < 0.0)

        self._controls['function1'] = (msg.buttons[6] == 1)
        self._controls['function2'] = (msg.buttons[10] == 1)


class LogitechController(Joystick):
    """
    Logitech specialization of Joystick.
    """
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        super(LogitechController, self).__init__(scale, offset, deadband)

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """

        self._controls['btnLeft'] = (msg.buttons[0] == 1)
        self._controls['btnUp'] = (msg.buttons[3] == 1)
        self._controls['btnDown'] = (msg.buttons[1] == 1)
        self._controls['btnRight'] = (msg.buttons[2] == 1)

        self._controls['dPadUp'] = (msg.axes[5] > 0.5)
        self._controls['dPadDown'] = (msg.axes[5] < -0.5)
        self._controls['dPadLeft'] = (msg.axes[4] > 0.5)
        self._controls['dPadRight'] = (msg.axes[4] < -0.5)

        self._controls['leftStickHorz'] = msg.axes[0]
        self._controls['leftStickVert'] = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[2]
        self._controls['rightStickVert'] = msg.axes[3]

        self._controls['leftBumper'] = (msg.buttons[4] == 1)
        self._controls['rightBumper'] = (msg.buttons[5] == 1)
        self._controls['leftTrigger'] = (msg.buttons[6] == 1)
        self._controls['rightTrigger'] = (msg.buttons[7] == 1)

        self._controls['function1'] = (msg.buttons[8] == 1)
        self._controls['function2'] = (msg.buttons[9] == 1)


class PS3Controller(Joystick):
    """
    PS3 specialization of Joystick.
    """
    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        super(PS3Controller, self).__init__(scale, offset, deadband)

    def _on_joy(self, msg):
        """ callback for messages from joystick input
        Args:
              msg(Joy): a joystick input message
        """

        self._controls['btnLeft'] = (msg.buttons[15] == 1)
        self._controls['btnUp'] = (msg.buttons[12] == 1)
        self._controls['btnDown'] = (msg.buttons[14] == 1)
        self._controls['btnRight'] = (msg.buttons[13] == 1)

        self._controls['dPadUp'] = (msg.axes[4] > 0.5)
        self._controls['dPadDown'] = (msg.axes[6] < -0.5)
        self._controls['dPadLeft'] = (msg.axes[7] > 0.5)
        self._controls['dPadRight'] = (msg.axes[5] < -0.5)

        self._controls['leftStickHorz'] = msg.axes[0]
        self._controls['leftStickVert'] = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[2]
        self._controls['rightStickVert'] = msg.axes[3]

        self._controls['leftBumper'] = (msg.buttons[10] == 1)
        self._controls['rightBumper'] = (msg.buttons[11] == 1)
        self._controls['leftTrigger'] = (msg.buttons[8] == 1)
        self._controls['rightTrigger'] = (msg.buttons[9] == 1)

        self._controls['function1'] = (msg.buttons[0] == 1)
        self._controls['function2'] = (msg.buttons[3] == 1)
