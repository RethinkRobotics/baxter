#!/usr/bin/env python
import argparse

import roslib
roslib.load_manifest('joint_pose')
import rospy
import enable_robot

from Mapper import Mapper
from JointPositionBaxterController import JointPositionBaxterController
from sensor_msgs.msg import Joy

class JoystickMapper(Mapper):
  """ Maps joystick input to robot control
  """


  def __init__(self, controller, padType):
    """ Maps joystick input to robot control
    Sets up the bindings
    Args:
      controller(BaxterController): a type of Baxter robot controller
      padType(str): the type of controller used ('xbox' or 'logitech')
    """
    super(JoystickMapper, self).__init__(controller)
    self.sub = rospy.Subscriber("/joy", Joy, self.incoming)
    self.controls = {}
    self.padType = padType
    self.newData = False
    self._setupBindings()

  def _setupBindings(self):
    """ private function to setup the bindings
    from generic joystick to robot command
    """
    class ButtonTransition(object):
      """ local class to monitor transitions
      The transition is measured when read
      """
      def __init__(self, controls, name, downVal=1, upVal=0):
        self.controls = controls
        self.name = name
        self.value = upVal
        self.downVal = downVal
        self.upVal = upVal
      def changed(self):
        if self.name in self.controls:
          newValue = self.controls[self.name]
        equal = (newValue == self.value)
        self.value = newValue
        return not equal
      def down(self):
        if (self.changed() and (self.value == self.downVal)):
          print ("button %s pressed" % (self.name))
          return True
        return False
      def up(self):
        return (self.changed() and (self.value == self.upVal))

    def createButtonChangedDict(*buttonNames):
      """ Creates a dictionary holding the transition
      objects  for each button
      """
      btnDict = {}
      for name in buttonNames:
        btnDict[name] = ButtonTransition(self.controls,name)
      return btnDict

    class JointSelector(object):
      """ Local class to maintain a selection of
      joints to control depending on the control mode
      """
      def __init__(self, *joints):
        self.joints = joints
        self.index = 0
      def inc(self):
        self.index = (self.index+1) % len(self.joints)
        print("selected %s" % (self.joints[self.index]))
      def dec(self):
        self.index = (self.index-1) % len(self.joints)
        print("selected %s" % (self.joints[self.index]))
      def get(self, offset=0):
        return self.joints[(self.index+offset)%len(self.joints)]

    buttons = createButtonChangedDict('rightBumper', 'leftBumper', 'function1', 'function2', 'leftTrigger', 'rightTrigger', 'btnDown', 'btnLeft', 'btnRight')
    leftSelector = JointSelector('left_s0','left_s1','left_e0','left_e1','left_w0', 'left_w1', 'left_w2')
    rightSelector = JointSelector('right_s0','right_s1','right_e0','right_e1','right_w0', 'right_w1', 'right_w2')

    def createTransFunction(transitions, function, useRetVal = False):
      def f(controlName):
        if transitions[controlName].down():
          retVal = function()
          if useRetVal:
            return retVal
        return (None, None)#to appease caller expecting a tuple
      return f

    def createCommandFunction(selector, offset, scale):
      """ function to create a command function
      Create a function to execute a specific control command
      Args:
        selector(JointSelector):holds joint names to control
        offset(int): modify which joint selector.get returns
        scale(float): scale factor for the command
      Returns: a function to control a specific joint
      """
      def f(controlName):
        """ function to control a specific joint
        Args:
          controlName(str): the name of a an input control to
          read, scale and store in a tupel
        Returns:
          a tuple containing a joint name and an associated
          command value
        """
        return (selector.get(offset), self.controls[controlName] * scale)
      return f

    self.bindings = {
      'btnDown':  createTransFunction(buttons, self.controller.record),
      'function1':  createTransFunction(buttons, self.stop),
      'function2':  createTransFunction(buttons, self.stop),
      'rightBumper':  createTransFunction(buttons, leftSelector.inc),
      'rightTrigger': createTransFunction(buttons, leftSelector.dec),
      'leftBumper':   createTransFunction(buttons, rightSelector.inc),
      'leftTrigger':  createTransFunction(buttons, rightSelector.dec),
      'rightStickHorz': createCommandFunction(leftSelector,0,-1),
      'rightStickVert': createCommandFunction(leftSelector,1,-1),
      'leftStickHorz':  createCommandFunction(rightSelector,0,-1),
      'leftStickVert':  createCommandFunction(rightSelector,1,-1),
      'btnLeft':  createTransFunction(buttons, self.gripLeft, True),
      'btnRight':  createTransFunction(buttons, self.gripRight, True),
    }

  def gripRight(self):
    """ ugly helper for gripping """
    if self.controller.gripperRight.position > 50.0:
      return ('right_gripper',0.0)
    else:
      return ('right_gripper',100.0)

  def gripLeft(self):
    """ ugly helper for gripping """
    if self.controller.gripperLeft.position > 50.0:
      return ('left_gripper',0.0)
    else:
      return ('left_gripper',100.0)

  def incoming(self, msg):
    """ callback for messages from joystick input
    Args:
       msg(Joy): a joystick input message
    """
    def deadband(axis,size):
      """ Local function to create a deadband
      Args:
        axis(float): the value of the to-be-deadbanded axis
        size(float): the size of the deadband
      Returns:
        the deadbanded value of the axis
      """
      if axis > size or axis < -size:
        return axis
      return 0

    if self.padType == "xbox":
      self.controls['btnLeft'] = (msg.buttons[2] == 1)
      self.controls['btnUp'] = (msg.buttons[3] == 1)
      self.controls['btnDown'] = (msg.buttons[0] == 1)
      self.controls['btnRight'] = (msg.buttons[1] == 1)

      self.controls['dPadUp'] = (msg.axes[7] > 0.5)
      self.controls['dPadDown'] = (msg.axes[7] < -0.5)
      self.controls['dPadLeft'] = (msg.axes[6] > 0.5)
      self.controls['dPadRight'] = (msg.axes[6] < -0.5)

      self.controls['leftStickHorz'] = deadband(msg.axes[0],0.1)
      self.controls['leftStickVert'] = deadband(msg.axes[1],0.1)
      self.controls['rightStickHorz'] = deadband(msg.axes[3],0.1)
      self.controls['rightStickVert'] = deadband(msg.axes[4],0.1)

      self.controls['leftBumper'] = (msg.buttons[4] == 1)
      self.controls['rightBumper'] = (msg.buttons[5] == 1)
      self.controls['leftTrigger'] = (msg.axes[2] > 0.5)
      self.controls['rightTrigger'] = (msg.axes[5] > 0.5)

      self.controls['function1'] = (msg.buttons[6] == 1)
      self.controls['function2'] = (msg.buttons[10] == 1)

    elif self.padType == "logitech":
      self.controls['btnLeft'] = (msg.buttons[0] == 1)
      self.controls['btnUp'] = (msg.buttons[3] == 1)
      self.controls['btnDown'] = (msg.buttons[1] == 1)
      self.controls['btnRight'] = (msg.buttons[2] == 1)

      self.controls['dPadUp'] = (msg.axes[5] > 0.5)
      self.controls['dPadDown'] = (msg.axes[5] < -0.5)
      self.controls['dPadLeft'] = (msg.axes[4] > 0.5)
      self.controls['dPadRight'] = (msg.axes[4] < -0.5)

      self.controls['leftStickHorz'] = deadband(msg.axes[0],0.1)
      self.controls['leftStickVert'] = deadband(msg.axes[1],0.1)
      self.controls['rightStickHorz'] = deadband(msg.axes[2],0.1)
      self.controls['rightStickVert'] = deadband(msg.axes[3],0.1)

      self.controls['leftBumper'] = (msg.buttons[4] == 1)
      self.controls['rightBumper'] = (msg.buttons[5] == 1)
      self.controls['leftTrigger'] = (msg.buttons[6] == 1)
      self.controls['rightTrigger'] = (msg.buttons[7] == 1)

      self.controls['function1'] = (msg.buttons[8] == 1)
      self.controls['function2'] = (msg.buttons[9] == 1)
    else:
      print("no bindings for joystick type %s" % self.padType)
      self.done = True
      raise OSError(EINVAL, "unknown padType")
    self.newData = True

  def loop(self):
    """ loops to translate joystick input into robot control
    """
    self.done = False
    print("listening for joystick commands")
    while not self.done:
      if self.newData:
        commands = {}
        for controlName, controlFunction in self.bindings.items():
          (joint_name, joint_pos) = controlFunction(controlName)
          """ control functions do not need to yield a command, and
          thus can leave joint_name 'None' to simply be executed
          """
          if joint_name:
             commands[joint_name] = joint_pos
        self.controller.command(commands)

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("joystick", help="specify the type of joystick to use; xbox or logitech")
  args = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node("rethink_rsdk_joint_pose_joystick")
  print("Getting robot state... ")
  rs = enable_robot.RobotState()
  print("Enabling robot... ")
  rs.enable()

  if args.joystick in ['xbox', 'logitech']:
    mapper = JoystickMapper(JointPositionBaxterController(), args.joystick)
  else:
    parser.error("Unsupported joystick type '%s'" % (options.joystick))

  mapper.loop()
  rs.disable()
