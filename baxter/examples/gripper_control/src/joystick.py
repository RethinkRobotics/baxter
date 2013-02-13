#!/usr/bin/env python
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
    self.joystick = joystick
    self.name = name
  def changed(self):
    raise NotImplementedError()



class Joystick:
  """ Handles joystick input
  """

  def __init__(self, pad_type, scale=0.5, offset= 0.5, deadband=0.01):
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
    self.pad_type = pad_type
    self.scale = scale
    self.offset = offset
    self.deadband = deadband
    self.controls = {}
    self.new_data = False

  def on_joy(self, msg):
    """ callback for messages from joystick input
    Args:
       msg(Joy): a joystick input message
    """
    def stick_value(value, deadband, scale, offset):
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

    if self.pad_type == "xbox":
      self.controls['btnLeft'] = (msg.buttons[2] == 1)
      self.controls['btnUp'] = (msg.buttons[3] == 1)
      self.controls['btnDown'] = (msg.buttons[0] == 1)
      self.controls['btnRight'] = (msg.buttons[1] == 1)

      self.controls['dPadUp'] = (msg.axes[7] > 0.5)
      self.controls['dPadDown'] = (msg.axes[7] < -0.5)
      self.controls['dPadLeft'] = (msg.axes[6] > 0.5)
      self.controls['dPadRight'] = (msg.axes[6] < -0.5)

      self.controls['leftStickHorz']  = stick_value(msg.axes[0], self.deadband, self.scale, self.offset)
      self.controls['leftStickVert']  = stick_value(msg.axes[1], self.deadband, self.scale, self.offset)
      self.controls['rightStickHorz'] = stick_value(msg.axes[3], self.deadband, self.scale, self.offset)
      self.controls['rightStickVert'] = stick_value(msg.axes[4], self.deadband, self.scale, self.offset)

      self.controls['leftBumper'] = (msg.buttons[4] == 1)
      self.controls['rightBumper'] = (msg.buttons[5] == 1)
      self.controls['leftTrigger'] = (msg.axes[2] > 0.5)
      self.controls['rightTrigger'] = (msg.axes[5] > 0.5)

      self.controls['function1'] = (msg.buttons[6] == 1)
      self.controls['function2'] = (msg.buttons[10] == 1)

    elif self.pad_type == "logitech":
      self.controls['btnLeft'] = (msg.buttons[0] == 1)
      self.controls['btnUp'] = (msg.buttons[3] == 1)
      self.controls['btnDown'] = (msg.buttons[1] == 1)
      self.controls['btnRight'] = (msg.buttons[2] == 1)

      self.controls['dPadUp'] = (msg.axes[5] > 0.5)
      self.controls['dPadDown'] = (msg.axes[5] < -0.5)
      self.controls['dPadLeft'] = (msg.axes[4] > 0.5)
      self.controls['dPadRight'] = (msg.axes[4] < -0.5)

      self.controls['leftStickHorz']  = stick_value(msg.axes[0], self.deadband, self.scale, self.offset)
      self.controls['leftStickVert']  = stick_value(msg.axes[1], self.deadband, self.scale, self.offset)
      self.controls['rightStickHorz'] = stick_value(msg.axes[2], self.deadband, self.scale, self.offset)
      self.controls['rightStickVert'] = stick_value(msg.axes[3], self.deadband, self.scale, self.offset)

      self.controls['leftBumper'] = (msg.buttons[4] == 1)
      self.controls['rightBumper'] = (msg.buttons[5] == 1)
      self.controls['leftTrigger'] = (msg.buttons[6] == 1)
      self.controls['rightTrigger'] = (msg.buttons[7] == 1)

      self.controls['function1'] = (msg.buttons[8] == 1)
      self.controls['function2'] = (msg.buttons[9] == 1)
    else:
      print("no bindings for joystick type %s" % self.pad_type)
      raise OSError(EINVAL, "unknown pad type")
    self.new_data = True


  def get_value(self, control, default=0):
      if control in self.controls:
          return self.controls[control]
      return default

  class ButtonTransition(Transition):
    """ local class to monitor transitions
    The transition is measured when read
    """
    def __init__(self, joystick, name, down_val=1, up_val=0):
      super(Joystick.ButtonTransition, self).__init__(joystick, name)
      self.value = up_val
      self.down_val = down_val
      self.up_val = up_val
    def changed(self):
      new_value = self.joystick.get_value(self.name, self.value)
      equal = (new_value == self.value)
      self.value = new_value
      return not equal
    def down(self):
      if (self.changed() and (self.value == self.down_val)):
        return True
      return False
    def up(self):
      return (self.changed() and (self.value == self.up_val))

  def create_button_changed_dict(self, *buttonNames):
      """ Creates a dictionary holding the transition
      objects  for each button
      """
      button_dict = {}
      for name in buttonNames:
          button_dict[name] = Joystick.ButtonTransition(self, name)
      return button_dict


  class StickTransition(Transition):
    """ local class to monitor transitions
    The transition is measured when read
    """
    def __init__(self, joystick, name, epsilon=0.001):
      super(Joystick.StickTransition, self).__init__(joystick, name)
      self.epsilon = epsilon
      self.value = 0.0
    def get_value(self):
      new_value = self.joystick.get_value(self.name, self.value)
      if abs(new_value) < self.epsilon:
        new_value = 0.0
      return new_value
    def changed(self):
      self.value = self.get_value()
      return self.value != 0.0

  def create_stick_changed_dict(self, *stickNames):
      """ Creates a dictionary holding the transition
      objects  for each joystick
      """
      stick_dict = {}
      for name in stickNames:
          stick_dict[name] = self.StickTransition(self, name)
      return stick_dict
