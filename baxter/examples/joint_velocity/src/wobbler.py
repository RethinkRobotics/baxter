#!/usr/bin/env python
import roslib
roslib.load_manifest('joint_velocity')
import rospy
import baxter_interface
import baxter_interface.limb
from std_msgs.msg import (UInt16,)
from sensor_msgs.msg import (JointState,)
from baxter_msgs.msg import (JointVelocities, JointCommandMode,)

import signal
import math
import random

class Wobbler():

    def __init__(self):
        # set joint state publishing to 1000Hz
        self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self.left_arm = baxter_interface.limb.Limb("left")
        self.right_arm = baxter_interface.limb.Limb("right")
        self.joint_names = ['s0','s1','e0','e1','w0','w1','w2',]

        self.pub_rate.publish(1000)
        self.done = False
        signal.signal(signal.SIGINT, self._handle_ctrl_c)

    def _handle_ctrl_c(self, signum, frame):
       print("stopping...")
       self.done = True

    def set_neutral(self):
        angles = [0, -0.55, 0, 1.28, 0, 0.26, 0]
        cmd = dict(zip(self.joint_names, angles))
        self.left_arm.set_pose(cmd)
        self.right_arm.set_pose(cmd)

    def wobble(self):
        self.set_neutral()
        rate = rospy.Rate(1000);
        start = rospy.Time.now()
        def make_v_func():
            period_factor = random.uniform(0.1, 0.3)
            amplitude_factor = random.uniform(0.1, 0.3)
            def v_func(elapsed):
                return math.cos(period_factor * elapsed.to_sec() * math.pi * 2) * amplitude_factor
            return v_func
        v_funcs = [make_v_func() for x in range(7)]
        while not self.done:
            self.pub_rate.publish(1000)
            elapsed = rospy.Time.now() - start
#            print("commanded %f, left %f, right %f, rate %f" % (
#                velocity,
#                self.left_arm.joint_velocity('s0'),
#                self.right_arm.joint_velocity('s0'),
#                self.left_arm.state_rate(),
#            ))
            cmd = dict(zip(self.joint_names, [v_funcs[i](elapsed) for i in range(7)]))
            self.left_arm.set_velocities(cmd)
            cmd = dict(zip(self.joint_names, [-v_funcs[i](elapsed) for i in range(7)]))
            self.right_arm.set_velocities(cmd)
            rate.sleep()
        #return to normal
        for i in range(1000):
            self.pub_rate.publish(100)
            rate.sleep()
        self.set_neutral()

if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_velocity", disable_signals=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    wobbler = Wobbler()
    wobbler.wobble()

    rs.disable()
    print("done.")
