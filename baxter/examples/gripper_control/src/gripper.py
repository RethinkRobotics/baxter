#!/usr/bin/env python
"""
Baxter RSDK Gripper Example
"""
from optparse import OptionParser

import roslib
roslib.load_manifest('gripper_control')
import rospy

import enable_robot
from joystick import Joystick
from controllers import GripperBaxterController
from mappers import JoystickMapper, KeyboardMapper

def gripper_main():
  """ Gripper example main()
      --joystick option operates the gripper from a game controller
      otherwise the gripper is operated by keyboard.
  """
  parser = OptionParser()
  parser.add_option("-j", "--joystick", dest="joystick", 
                    help="specify the type of joystick to use; xbox or logitech")
  (options, args) = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node('gripper')
  print("Getting robot state... ")
  rs = enable_robot.RobotState()
  print("Enabling robot... ")
  rs.enable()

  controller = GripperBaxterController()

  if options.joystick and not options.joystick.lower() == 'none':
    if options.joystick in ['xbox', 'logitech']:
      joystick = Joystick(options.joystick)
      mapper = JoystickMapper(controller, joystick)
    else:
      parser.error("Unsupported joystick type '%s'" % (options.joystick))
      return 1
  else:
    mapper = KeyboardMapper(controller)

  mapper.run()
  rs.disable()
  return 0

if __name__ == '__main__':
  exit(gripper_main())
