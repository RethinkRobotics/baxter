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
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()

        # set joint state publishing to 100Hz
        self._pub_rate.publish(100)

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose

        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling of both arms

        """
        rate = rospy.Rate(100);
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

        v_funcs = [make_v_func() for x in range(len(self._right_joint_names))]
        done = False
        print("Wobbling. Press any key to stop...")
        while not done and not rospy.is_shutdown():
            if iodevices.getch():
                done = True
            else:
                self._pub_rate.publish(100)
                elapsed = rospy.Time.now() - start
                cmd = dict(zip(self._left_joint_names, [v_funcs[i](elapsed) for i in range(len(self._left_joint_names))]))
                self._left_arm.set_joint_velocities(cmd)
                cmd = dict(zip(self._right_joint_names, [-v_funcs[i](elapsed) for i in range(len(self._right_joint_names))]))
                self._right_arm.set_joint_velocities(cmd)
                rate.sleep()

        rate = rospy.Rate(100);
        if not rospy.is_shutdown():
            for i in range(100):
                if rospy.is_shutdown():
                    return False
                self._left_arm.set_joint_position_mode()
                self._right_arm.set_joint_position_mode()
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
