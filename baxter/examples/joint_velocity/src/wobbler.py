#!/usr/bin/env python
import signal
import math
import random

import roslib
roslib.load_manifest('joint_velocity')
import rospy
import baxter_interface
import baxter_interface.limb

from std_msgs.msg import (UInt16,)
from sensor_msgs.msg import (JointState,)
from baxter_msgs.msg import (JointVelocities,
                             JointCommandMode,
                            )

class Wobbler():

    def __init__(self):
        """
        'Wobbles' both arms by driving the joint velocities to sinusoid functions

        """
        self._pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._joint_names = ['s0','s1','e0','e1','w0','w1','w2',]

        # set joint state publishing to 1000Hz
        self._pub_rate.publish(1000)
        self._done = False
        signal.signal(signal.SIGINT, self._handle_ctrl_c)

    def _handle_ctrl_c(self, signum, frame):
       print("stopping...")
       self._done = True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose

        """
        angles = [0, -0.55, 0, 1.28, 0, 0.26, 0]
        cmd = dict(zip(self._joint_names, angles))
        self._left_arm.set_pose(cmd)
        self._right_arm.set_pose(cmd)

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms

        """
        rate = rospy.Rate(1000);
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cos function to control a specific joint

            """
            period_factor = random.uniform(0.3, 0.5)
            amplitude_factor = random.uniform(0.1, 0.2)
            def v_func(elapsed):
                return math.cos(period_factor * elapsed.to_sec() * math.pi * 2) * amplitude_factor
            return v_func

        v_funcs = [make_v_func() for x in range(len(self._joint_names))]
        while not self._done:
            self._pub_rate.publish(1000)
            elapsed = rospy.Time.now() - start
            cmd = dict(zip(self._joint_names, [v_funcs[i](elapsed) for i in range(len(self._joint_names))]))
            self._left_arm.set_velocities(cmd)
            cmd = dict(zip(self._joint_names, [-v_funcs[i](elapsed) for i in range(len(self._joint_names))]))
            self._right_arm.set_velocities(cmd)
            rate.sleep()

        #return to normal
        self.set_neutral()
        for i in range(100):
            self._pub_rate.publish(100)
            rate.sleep()

if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_velocity")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    wobbler = Wobbler()
    print("Wobbling... ")
    wobbler.wobble()

    print("Disabling robot... ")
    rs.disable()
    print("done.")
