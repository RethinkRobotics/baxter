#!/usr/bin/env python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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

        if not rospy.is_shutdown():
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
    wobbler.wobble()

    print("Disabling robot... ")
    rs.disable()
    print("done.")
