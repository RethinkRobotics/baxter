#!/usr/bin/env python
import signal
import math
import random

import roslib
roslib.load_manifest('joint_velocity')
import rospy

from std_msgs.msg import (
    UInt16,)
from sensor_msgs.msg import (
    JointState,)
from baxter_msgs.msg import (
    JointVelocities,
    JointCommandMode,)

import baxter_interface
import iodevices

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

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose

        """
        print("Moving to neutral pose...")
        self._left_arm.set_neutral_pose()
        self._right_arm.set_neutral_pose()

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
        done = False
        print("Wobbling. Press any key to stop...")
        while not done and not rospy.is_shutdown():
            if iodevices.getch():
                done = True
            else:
                self._pub_rate.publish(1000)
                elapsed = rospy.Time.now() - start
                cmd = dict(zip(self._joint_names, [v_funcs[i](elapsed) for i in range(len(self._joint_names))]))
                self._left_arm.set_velocities(cmd)
                cmd = dict(zip(self._joint_names, [-v_funcs[i](elapsed) for i in range(len(self._joint_names))]))
                self._right_arm.set_velocities(cmd)
                rate.sleep()

        rate = rospy.Rate(100);
        if not rospy.is_shutdown():
            for i in range(100):
                if rospy.is_shutdown():
                    return False
                self._left_arm.set_position_mode()
                self._right_arm.set_position_mode()
                self._pub_rate.publish(100)
                rate.sleep()
            #return to normal
            self.set_neutral()
            return True

if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_velocity")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    wobbler = Wobbler()
    wobbler.wobble()

    print("Disabling robot... ")
    rs.disable()
    print("done.")
