import sys
import time

import roslib
roslib.load_manifest('baxter_interface')
import rospy

import baxter_msgs.msg
import sensor_msgs.msg

class Hand(object):
    def __init__(self, limb):
        """
        Interface class for a hand on the Baxter robot.

        @param limb - limb to interface
        """
        self.name = limb
 
        ns = '/robot/limb/' + limb + '/'

        self.pub_enable = rospy.Publisher(self.base_path + 'set_enabled', Bool)
        self.pub_reset = rospy.Publisher(self.base_path + 'command_reset', Bool)
        self.pub_calibrate = rospy.Publisher(
          self.base_path + 'command_calibrate', Empty)
        self.pub_actuate = rospy.Publisher(
          self.base_path + 'command_set', GripperCommand)

        self.sub_identity = rospy.Subscriber(
          self.base_path + 'identity', GripperIdentity, self.on_gripper_identity)
        self.sub_properties = rospy.Subscriber(
          self.base_path + 'properties', GripperProperties, self.on_gripper_properties)
        self.sub_state = rospy.Subscriber(
          self.base_path + 'state', GripperState, self.on_gripper_state)


