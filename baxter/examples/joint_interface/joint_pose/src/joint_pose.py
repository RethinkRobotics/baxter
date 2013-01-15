#!/usr/bin/env python
import os.path
import sys
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
from baxter_msgs.msg import JointControlMode #TODO: rename to JointCommandMode on both sides
from baxter_msgs.msg import JointPositions


class JointController(object):
  """ Abstract base class for robot joint controllers """
  def command(self, jointName, delta):
    raise NotImplementedError()

class BaxterController(JointController):
  """
    Joint state should not be split by arm; instead all joints on the robot should be included as defined in the URDF
  """
  def __init__(self, outputFilename):
    super( BaxterController, self).__init__()
    self.pubLeftMode = rospy.Publisher('/robot/limb/left/joint_command_mode', JointControlMode)
    self.pubRightMode = rospy.Publisher('/robot/limb/right/joint_command_mode', JointControlMode)
    self.pubLeft = rospy.Publisher('/robot/limb/left/command_joint_angles', JointPositions)
    self.pubRight = rospy.Publisher('/robot/limb/right/command_joint_angles', JointPositions)
    self.subLeft = rospy.Subscriber('/robot/limb/left/joint_states', JointState, self.leftJointState)
    self.subRight = rospy.Subscriber('/robot/limb/right/joint_states', JointState, self.rightJointState)
    self.leftPosition = {}
    self.rightPosition = {}
    self.outputFilename = outputFilename
    self.newFile = True
    self.setPositionMode()

  def setPositionMode(self):
    msg = JointControlMode()
    msg.data = JointControlMode.POSITION
    self.pubLeftMode.publish(msg)
    self.pubRightMode.publish(msg)


  def record(self):
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
    for i in range(len(data.name)):
      self.leftPosition['left_'+data.name[i]] = data.position[i]

  def rightJointState(self, data):
    for i in range(len(data.name)):
      self.rightPosition['right_'+data.name[i]] = data.position[i]

  def command(self, jointDeltas, posIsDelta=True):
    msg = JointPositions()
    for jointName, delta in jointDeltas.items():
      if jointName in self.leftPosition:
        if posIsDelta:
          self.leftPosition[jointName] += delta
        else:
          self.leftPosition[jointName] = delta
      elif jointName in self.rightPosition:
        if posIsDelta:
          self.rightPosition[jointName] += delta
        else:
          self.rightPosition[jointName] = delta
      else:
        raise OSError(EINVAL, "unknown jointname %s" % (jointName))

    msg.names = self.leftPosition.keys()
    msg.angles = self.leftPosition.values()
    self.pubLeft.publish(msg)

    msg.names = self.rightPosition.keys()
    msg.angles = self.rightPosition.values()
    self.pubRight.publish(msg)


class FileMapper(object):
  def __init__(self, controller, filename, rate):
    self.controller = controller
    self.filename = filename
    self.rate = rate

  def loop(self):
    print "playing back %s @ %dHz" % (self.filename, self.rate)
    with open(self.filename, 'r') as f:
      lines = f.readlines()
    keys = lines[0].rstrip().split(',')
    for values in lines[1:]:
      print values
      values = [float(x) for x in values.rstrip().split(',')]
      self.controller.command(dict(zip(keys, values)), False)
      time.sleep(1.0/self.rate)

class JoystickMapper(object):
  def __init__(self, controller, padType):
    self.sub = rospy.Subscriber("/joy", Joy, self.incoming)
    self.controller = controller
    self.controls = {}
    self.padType = padType
    self.newData = False

    cf = self.createCommandFunction
    self.bindings = {
      'rightStickHorz': [cf('left_s0',-1), cf('left_e0',-1), cf('left_w0',-1)],
      'rightStickVert': [cf('left_s1',-1), cf('left_e1',-1), cf('left_w1',-1)],
      'leftStickHorz':  [cf('right_s0',-1), cf('right_e0',-1), cf('right_w0',-1)],
      'leftStickVert':  [cf('right_s1',-1), cf('right_e1',-1), cf('right_w1',-1)]
    }

  def createCommandFunction(self, joint_name, scale):
    def f(controlName):
      return (joint_name, self.controls[controlName] * scale)
    return f

  def incoming(self, msg):
    if self.padType == "xbox":
      self.controls['x'] = (msg.buttons[2] == 1)
      self.controls['y'] = (msg.buttons[3] == 1)
      self.controls['a'] = (msg.buttons[0] == 1)
      self.controls['b'] = (msg.buttons[1] == 1)
      self.controls['dPadUp'] = (msg.axes[7] > 0.5)
      self.controls['dPadDown'] = (msg.axes[7] < -0.5)
      self.controls['dPadLeft'] = (msg.axes[6] > 0.5)
      self.controls['dPadRight'] = (msg.axes[6] < -0.5)
      self.controls['leftBumper'] = (msg.buttons[4] == 1)
      self.controls['rightBumper'] = (msg.buttons[5] == 1)
      self.controls['leftStickHorz'] = msg.axes[0]
      self.controls['leftStickVert'] = msg.axes[1]
      self.controls['leftTrigger'] = msg.axes[2]
      self.controls['rightStickHorz'] = msg.axes[3]
      self.controls['rightStickVert'] = msg.axes[4]
      self.controls['rightTrigger'] = msg.axes[5]
      self.controls['back'] = (msg.buttons[6] == 1)
    else:
      raise OSError(EINVAL, "unknown padType")
    self.newData = True

  def loop(self):
    self.done = False
    mode = 0
    lastControls = self.controls.copy()
    while not self.done:
      if self.newData:
        commands = {}
        for key, value in self.bindings.items():
          (joint_name, joint_pos) = value[mode % len(value)](key)
          if joint_name:
            commands[joint_name] = joint_pos
        if self.controls['rightBumper'] and not lastControls['rightBumper']:
          mode += 1
        if self.controls['leftBumper'] and not lastControls['leftBumper']:
          mode -= 1
        if self.controls['a'] and not lastControls['a']:
          self.controller.record()
        self.done = self.controls['back']
        self.controller.command(commands)
        lastControls = self.controls.copy()

class KeyboardMapper(object):
  """ class that listens to keypresses and sends associated robot joint commands """

  def __init__(self, jointController):
    self.mode = 0
    self.done = False
    self.jointController = jointController
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
    self.jointController.record()

  def stop(self):
    """ Esc: stop """
    self.done = True

  def createCommandFunction(self, jointName, delta):
    """create a function to increment a specific joint by a specific delta"""
    def commandFunction():
      try:
        self.jointController.command({jointName: delta})
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
    for key, cmds in self.bindings.iteritems():
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
  parser.add_option("-j", "--joystick", dest="joystick", action="store_true", default=False, help="use joystick for input")
  parser.add_option("-k", "--keyboard", dest="keyboard", action="store_true", default=True, help="use keyboard for input")
  parser.add_option("-o", "--output",   dest="outputFilename", help="filename for output")
  parser.add_option("-i", "--input",   dest="inputFilename", help="filename for playback")
  parser.add_option("-r", "--rate",   dest="rate", type="int", default=30,  help="rate for playback")
  (options, args) = parser.parse_args()

  if options.joystick or options.keyboard:
    rospy.init_node('posejoint')
    rs = enable_robot.RobotState()
    rs.enable()

    controller = BaxterController(options.outputFilename)
    if options.inputFilename:
      mapper = FileMapper(controller, options.inputFilename, options.rate)
    elif options.joystick:
      mapper = JoystickMapper(controller, "xbox")
    else:
      mapper = KeyboardMapper(controller)
    mapper.loop()

    rs.disable()
  else:
    parser.error("use either keyboard (default) or joystick")
