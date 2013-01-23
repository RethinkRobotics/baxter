#!/usr/bin/env python
import os.path
import sys
import signal
import termios
import time
import tty
from optparse import OptionParser
from errno import EINVAL

import roslib
roslib.load_manifest('joint_pose')
import rospy
import enable_robot

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from baxter_msgs.msg import JointCommandMode
from baxter_msgs.msg import JointPositions

class BaxterController(object):
  """ Interface for a Baxter robot controller
  """


  def command(self, commands):
    """ generic command function
    Args:
      commands(dict{str:int}): a set of name/value pairs
        that forms a robot command
    """
    raise NotImplementedError()


  def record(self):
    """ records the last issued command or robot state
    """
    raise NotImplementedError()



class JointPositionBaxterController(BaxterController):
  """ Joint Position Controller for a Rethink Robotics Baxter RSDK Robot
  Controls a baxter robot by setting joint angles by combining joint angles
  read from the robot with user commands
  """


  def __init__(self, outputFilename):
    """ Services subscribers and publishers for communication with Baxter
    Specifically for Joint Position Control
    Args:
      outputFileName(str):optional, when provided can be used to record joint
      positions
    """
    super(JointPositionBaxterController, self).__init__()
    self.pubLeftMode = rospy.Publisher('/robot/limb/left/joint_command_mode', JointCommandMode)
    self.pubRightMode = rospy.Publisher('/robot/limb/right/joint_command_mode', JointCommandMode)
    self.pubLeft = rospy.Publisher('/robot/limb/left/command_joint_angles', JointPositions)
    self.pubRight = rospy.Publisher('/robot/limb/right/command_joint_angles', JointPositions)
    self.subLeft = rospy.Subscriber('/robot/limb/left/joint_states', JointState, self.leftJointState)
    self.subRight = rospy.Subscriber('/robot/limb/right/joint_states', JointState, self.rightJointState)
    self.leftPosition = {}
    self.rightPosition = {}
    self.outputFilename = outputFilename
    self.newFile = True


  def setPositionMode(self):
    """  Set Baxter's joint command mode to Position Control
    publishes the desire to put the robot in position control mode
    """
    msg = JointCommandMode()
    msg.mode = JointCommandMode.POSITION
    self.pubLeftMode.publish(msg)
    self.pubRightMode.publish(msg)


  def record(self):
    """ Records the current joint positions to a csv file
    if outputFilename was provided at construction
    this function will record the latest set of joint angles
    in a csv format.
    This function does not test to see if a file exists and
    will overwrite existing files.
    """
    if self.outputFilename:
      if self.newFile:
        with open(self.outputFilename, 'w') as f:
          f.write(','.join(self.leftPosition.keys()) + ',')
          f.write(','.join(self.rightPosition.keys()) + '\n')
        self.newFile = False

      with open(self.outputFilename, 'a') as f:
        f.write(','.join([str(x) for x in self.leftPosition.values()]) + ',')
        f.write(','.join([str(x) for x in self.rightPosition.values()]) + '\n')


  def leftJointState(self, data):
    """ callback function for the ROS subscriber of the left joint angles
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      self.leftPosition['left_'+data.name[i]] = data.position[i]


  def rightJointState(self, data):
    """ callback function for the ROS subscriber of the right joint angles
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      self.rightPosition['right_'+data.name[i]] = data.position[i]


  def command(self, jointPositions, posIsDelta=True):
    """ Joint command function
    Publishes new joint angles based on the supplied deltas
    and the most recently received joint angle state.
    Args:
      jointPositions(dict{str:int}): a set of name/value pairs
        each representing a joint name and a joint position
        A subset of joints can be provided, but each joint must
        have appeared in a JointState msg (even for absolute cmds)
    Kwargs:
      posIsDelta(bool): when defaulted to True, interpret the positions
        in jointPositions as deltas from the current position.
        when set to False, use the positions as absolute
    """
    leftMsg = JointPositions()
    rightMsg = JointPositions()
    for jointName, pos in jointPositions.items():
      if jointName in self.leftPosition:
        leftMsg.names.append(jointName)
        if posIsDelta:
          leftMsg.angles.append(self.leftPosition[jointName] + pos)
        else:
          leftMsg.angles.append(pos)
      elif jointName in self.rightPosition:
        rightMsg.names.append(jointName)
        if posIsDelta:
          rightMsg.angles.append(self.rightPosition[jointName] + pos)
        else:
          rightMsg.angles.append(pos)

    self.setPositionMode()
    self.pubLeft.publish(leftMsg)
    self.pubRight.publish(rightMsg)




class Mapper(object):
  """ Interface class to map an input device to a controller
  """


  def __init__(self, controller):
    """ set up the ctrl-c handler for this input device
    Args:
      controller(
    """
    self.controller = controller
    signal.signal(signal.SIGINT, self.handleCtrlC)


  def handleCtrlC(self, signum, frame):
    """ Generic and crude ctrl-c handler
    Exits by using sys.exit.
    Recommended to override for a cleaner exit
    """
    print("Ctrl-C: exiting...")
    sys.exit(0)


  def loop(self):
    """ Virtual loop function
      function that loops while mapping
      input to control
    """
    raise NotImplementedError()



class FileMapper(Mapper):
  """ CSV File mapper
  Maps input read from a csv file at a fixed rate to
  robot control
  """


  def __init__(self, controller, filename, rate):
    """ Maps csv file input to robot control
    Args:
      controller(BaxterController): a type of Baxter robot controller
      filename(str): path to csv file to read from
      rate(int): a rate in Hz to command each line of input at
    """
    super(FileMapper, self).__init__(controller)
    self.filename = filename
    self.rate = rate

  def loop(self):
    """ Loops through csv file
    Does not loop indefinitely, but only until the file is read
    and processed. Reads each line, split up in columns and
    formats each line into a controller command in the form of
    name/value pairs. Names come from the column headers
    """
    print("playing back %s @ %dHz" % (self.filename, self.rate))
    with open(self.filename, 'r') as f:
      lines = f.readlines()
    keys = lines[0].rstrip().split(',')
    for values in lines[1:]:
      print(values)
      values = [float(x) for x in values.rstrip().split(',')]
      self.controller.command(dict(zip(keys, values)), False)
      time.sleep(1.0/self.rate)



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

  def handleCtrlC(self, signum, frame):
    """ ctrl-c handler
    """
    self.stop()

  def stop(self):
    print("Exiting...")
    self.done = True

  def _setupBindings(self):
    """ private function to setup the bindings
    from generic joystick to robot command
    """
    class ButtonTransition(object):
      """ local class to monitor transitions
      The transition is measured when read, so
      you can't miss them, but they could be stale
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

    buttons = createButtonChangedDict('rightBumper', 'leftBumper', 'start', 'leftTrigger', 'rightTrigger', 'btnDown')
    leftSelector = JointSelector('left_s0','left_s1','left_e0','left_e1','left_w0', 'left_w1', 'left_w2')
    rightSelector = JointSelector('right_s0','right_s1','right_e0','right_e1','right_w0', 'right_w1', 'right_w2')

    def createTransFunction(transitions, function):
      def f(controlName):
        if transitions[controlName].down():
          function()
        return (None, None) #to appease caller expecting a tuple
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
      'start':  createTransFunction(buttons, self.stop),
      'rightBumper':  createTransFunction(buttons, leftSelector.inc),
      'rightTrigger': createTransFunction(buttons, leftSelector.dec),
      'leftBumper':   createTransFunction(buttons, rightSelector.inc),
      'leftTrigger':  createTransFunction(buttons, rightSelector.dec),
      'rightStickHorz': createCommandFunction(leftSelector,0,-1),
      'rightStickVert': createCommandFunction(leftSelector,1,-1),
      'leftStickHorz':  createCommandFunction(rightSelector,0,-1),
      'leftStickVert':  createCommandFunction(rightSelector,1,-1),
    }


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

      self.controls['start'] = (msg.buttons[6] == 1)

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
      self.controls['leftTrigger'] = (msg.buttons[7] == 1)
      self.controls['rightTrigger'] = (msg.buttons[8] == 1)

      self.controls['start'] = (msg.buttons[6] == 1)
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

class KeyboardMapper(Mapper):
  """ class that listens to keypresses and sends associated robot joint commands """

  def __init__(self, controller):
    super(KeyboardMapper, self).__init__(controller)
    self.mode = 0
    self.done = False
    jcf = self.createCommandFunction

    # these deltas should be some pct of the range from the URDF
    # it would be nice if the whole mapping could be autogenerated from the URDF
    self.bindings = {
      #     mode 0: all left     mode 1: all right     mode 2: top l&r       mode 3: bottom l&r
      'a': [jcf('left_s0',+0.1), jcf('right_s0',+0.1), jcf('right_s0',+0.1), jcf('right_e1',+0.1)],
      'f': [jcf('left_s0',-0.1), jcf('right_s0',-0.1), jcf('right_s0',-0.1), jcf('right_e1',-0.1)],
      's': [jcf('left_s1',+0.1), jcf('right_s1',+0.1), jcf('right_s1',+0.1), jcf('right_w0',+0.1)],
      'd': [jcf('left_s1',-0.1), jcf('right_s1',-0.1), jcf('right_s1',-0.1), jcf('right_w0',-0.1)],
      'w': [jcf('left_w0',+0.1), jcf('right_w0',+0.1), jcf('right_e0',+0.1), jcf('right_w1',+0.1)],
      'e': [jcf('left_w0',-0.1), jcf('right_w0',-0.1), jcf('right_e0',-0.1), jcf('right_w1',-0.1)],

      'h': [jcf('left_e0',+0.1), jcf('right_e0',+0.1), jcf('left_s0', +0.1), jcf('left_e1', +0.1)],
      'l': [jcf('left_e0',-0.1), jcf('right_e0',-0.1), jcf('left_s0', -0.1), jcf('left_e1', -0.1)],
      'j': [jcf('left_e1',+0.1), jcf('right_e1',+0.1), jcf('left_s1', +0.1), jcf('left_w0', +0.1)],
      'k': [jcf('left_e1',-0.1), jcf('right_e1',-0.1), jcf('left_s1', -0.1), jcf('left_w0', -0.1)],
      'u': [jcf('left_w1',+0.1), jcf('right_w1',+0.1), jcf('left_e0', +0.1), jcf('left_w1', +0.1)],
      'i': [jcf('left_w1',-0.1), jcf('right_w1',-0.1), jcf('left_e0', -0.1), jcf('left_w1', -0.1)],

      'r': [jcf('left_w2',+0.1), jcf('right_w2',+0.1), jcf('right_w2',+0.1), jcf('right_w2',-0.1)],
      'y': [jcf('left_w2',-0.1), jcf('right_w2',-0.1), jcf('left_w2', +0.1), jcf('left_w2', -0.1)],

      'g': [self.incmode],
      ';': [self.decmode],
      '?': [self.showHelp],
      ' ': [self.record],
      '\x1b': [self.stop], #Escape... doesn't print.
    }

    self.showHelp()

  def record(self):
    self.controller.record()

  def stop(self):
    """ Esc: stop """
    self.done = True

  def createCommandFunction(self, jointName, delta):
    """create a function to increment a specific joint by a specific delta"""
    def commandFunction():
      try:
        self.controller.command({jointName: delta})
      except OSError:
        print "joint %s not found; is the robot running?" % (jointName)
    commandFunction.__doc__ = "modify " + jointName + " by " + str(delta)
    return commandFunction

  def getch(self):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
    except:
      raise OSError
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

  def incmode(self):
    """increment mode: switch key bindings"""
    self.mode += 1

  def decmode(self):
    """decrement mode: switch key bindings"""
    self.mode -= 1

  def showHelp(self):
    """show binding help"""
    print "============ bindings for current mode " + str(self.mode) + " =================="
    for key, cmds in sorted(self.bindings.items()):
      i = self.mode % len(cmds)
      print "    " + str(key) + ": " + str(cmds[i].__doc__)

  def execBinding(self,c):
    if c in self.bindings:
      cmds = self.bindings[c]
      i = self.mode % len(cmds)
      print cmds[i].__doc__
      cmds[i]()
    else:
      print "unknown key: " + c
      print "press '?' for help"

  def loop(self):
    self.done = False
    self.mode = 0
    while not self.done:
      c = self.getch()
      self.execBinding(c)

if __name__ == '__main__':
  parser = OptionParser()
  parser.add_option("-j", "--joystick", dest="joystick", help="specify the type of joystick to use; xbox or logitech")
  parser.add_option("-o", "--output",   dest="outputFilename", help="filename for output")
  parser.add_option("-i", "--input",   dest="inputFilename", help="filename for playback")
  parser.add_option("-r", "--rate",   dest="rate", type="int", default=30,  help="rate for playback")
  (options, args) = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node('posejoint')
  print("Getting robot state... ")
  rs = enable_robot.RobotState()
  print("Enabling robot... ")
  rs.enable()

  controller = JointPositionBaxterController(options.outputFilename)
  if options.inputFilename:
    mapper = FileMapper(controller, options.inputFilename, options.rate)
  elif options.joystick and not options.joystick.lower() == 'none':
    if options.joystick in ['xbox', 'logitech']:
      mapper = JoystickMapper(controller, options.joystick)
    else:
      parser.error("Unsupported joystick type '%s'" % (options.joystick))
  else:
    mapper = KeyboardMapper(controller)

  mapper.loop()
  rs.disable()
