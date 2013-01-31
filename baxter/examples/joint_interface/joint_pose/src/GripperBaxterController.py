import roslib
roslib.load_manifest('joint_pose')
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Float32
from baxter_msgs.msg import GripperState
from BaxterController import BaxterController

class GripperBaxterController(BaxterController):
  """ Controls a gripper on a Baxter Robot
  """
  def __init__(self, arm):
    self.pubCalibrate = rospy.Publisher('/robot/limb/' + arm + '/accessory/gripper/command_calibrate', Empty)
    self.pubGoto = rospy.Publisher('/robot/limb/' + arm + '/accessory/gripper/command_goto', Float32)
    self.subState = rospy.Subscriber('/robot/limb/' + arm + 'accessory/gripper/state', GripperState, self.gripperState)
    self.arm = arm
    self.calibrated = False
    self.position = 100.0 #open
    self.force = 0.0

  def gripperState(self, msg):
    self.position = msg.position
    self.force = msg.force
    self.calibrated = msg.calibrated

  def command(self, commands):
    if not self.calibrated:
      print("Calibrating %s gripper" % (self.arm,))
      self.pubCalibrate.publish(Empty())
    else:
      self.calibrating = False
      if 'position' in commands:
        position = commands['position']
        self.pubGoto.publish(Float32(position))
        self.position = position
      else:
        print("gripper %s does not know how to handle command(s) '%s'. Ignored" % (self.arm, str(commands.keys())))


