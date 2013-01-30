import roslib
roslib.load_manifest('joint_pose')
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Float32
from BaxterController import BaxterController

class GripperBaxterController(BaxterController):
  """ Controls a gripper on a Baxter Robot
  """
  def __init__(self, arm):
    self.pubCalibrate = rospy.Publisher('/robot/limb/' + arm + '/accessory/gripper/command_calibrate', Empty)
    self.pubGoto = rospy.Publisher('/robot/limb/' + arm + '/accessory/gripper/command_goto', Float32)
    self.calibrated = rospy.Time() #assumption, should check gripper/state, but don't have access right now
    self.position = 100.0 #assumption, should check gripper/state, but don't have access right now
    self.arm = arm

  def command(self, commands):
    if not self.calibrated:
      print("Calibrating %s gripper" % (self.arm,))
      self.pubCalibrate.publish(Empty())
      self.calibrated = rospy.Time.now()
    elif rospy.Time.now() - self.calibrated < rospy.Duration.from_sec(5.0):
      pass #still calibrating
    else:
      if 'position' in commands:
        position = commands['position']
        self.pubGoto.publish(Float32(position))
        self.position = position
      else:
        print("gripper %s does not know how to handle command(s) '%s'. Ignored" % (self.arm, str(commands.keys())))


