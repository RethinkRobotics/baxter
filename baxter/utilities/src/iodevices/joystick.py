#!/usr/bin/env python
import roslib
import rospy

from sensor_msgs.msg import Joy

class ButtonTransition(object):
    """ monitor button transitions
    The transition is measured when read
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
    """ monitor transitions in stick movement
    The transition is measured when read
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

    def decreased (self):
        value = self._raw_value()
        if (self._value - value) > self._epsilon:
            self._value = value
            return True
        return False

class Joystick(object):
    """ Abstract base class to handle joystick input
    """

    def __init__(self, scale=1.0, offset=0.0, deadband=0.1):
        """ Maps joystick input to robot control
        Sets up the bindings
        Args:
            scale(float): scaling applied to joystick values.
                                        raw joystick valuess are in [1.0...-1.0]
            offset(float): offset applied to joystick values, post-scaling
            deadband(float): deadband applied to scaled, offset values

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
    """ Xbox specialization of Joystick
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

        self._controls['leftStickHorz']  = msg.axes[0]
        self._controls['leftStickVert']  = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[3]
        self._controls['rightStickVert'] = msg.axes[4]

        self._controls['leftBumper'] = (msg.buttons[4] == 1)
        self._controls['rightBumper'] = (msg.buttons[5] == 1)
        self._controls['leftTrigger'] = (msg.axes[2] < 0.0)
        self._controls['rightTrigger'] = (msg.axes[5] < 0.0)

        self._controls['function1'] = (msg.buttons[6] == 1)
        self._controls['function2'] = (msg.buttons[10] == 1)

class LogitechController(Joystick):
    """ Logitech specialization of Joystick
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

        self._controls['leftStickHorz']  = msg.axes[0]
        self._controls['leftStickVert']  = msg.axes[1]
        self._controls['rightStickHorz'] = msg.axes[2]
        self._controls['rightStickVert'] = msg.axes[3]

        self._controls['leftBumper'] = (msg.buttons[4] == 1)
        self._controls['rightBumper'] = (msg.buttons[5] == 1)
        self._controls['leftTrigger'] = (msg.buttons[6] == 1)
        self._controls['rightTrigger'] = (msg.buttons[7] == 1)

        self._controls['function1'] = (msg.buttons[8] == 1)
        self._controls['function2'] = (msg.buttons[9] == 1)
