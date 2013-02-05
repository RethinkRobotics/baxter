#!/usr/bin/env python
from optparse import OptionParser

import roslib
roslib.load_manifest('gripper_control')
import rospy

import enable_robot
from joystick import Joystick
from controllers import GripperBaxterController
from mappers import JoystickMapper, KeyboardMapper, FileMapper


if __name__ == '__main__':
  parser = OptionParser()
  parser.add_option("-j", "--joystick", dest="joystick", help="specify the type of joystick to use; xbox or logitech")
  parser.add_option("-o", "--output",   dest="outputFilename", help="filename for output")
  parser.add_option("-i", "--input",   dest="inputFilename", help="filename for playback")
  (options, args) = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node('gripper')
  print("Getting robot state... ")
  rs = enable_robot.RobotState()
  print("Enabling robot... ")
  rs.enable()

  controller = GripperBaxterController()

  if options.inputFilename:
    mapper = FileMapper(controller, options.inputFilename)
  elif options.joystick and not options.joystick.lower() == 'none':
    if options.joystick in ['xbox', 'logitech']:
      joystick = Joystick(options.joystick)
      mapper = JoystickMapper(controller, joystick)
    else:
      parser.error("Unsupported joystick type '%s'" % (options.joystick))
  else:
    mapper = KeyboardMapper(controller)

  mapper.run()
  rs.disable()
