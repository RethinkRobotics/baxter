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

import argparse
import sys

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

class Puppeteer(object):

    def __init__(self, limb, amplification=1.0):
        """
        Puppets one arm with the other
        @param limb - the arm to be puppeted with the other
        @param amplification - factor by which to amplify the arm movement
        """
        puppet_arm = {"left":"right", "right":"left"}
        self._control_limb = limb
        self._puppet_limb = puppet_arm[limb]
        self._control_arm = baxter_interface.limb.Limb(self._control_limb)
        self._puppet_arm = baxter_interface.limb.Limb(self._puppet_limb)
        self._amp = amplification

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose

        """
        print("Moving to neutral pose...")
        self._control_arm.move_to_neutral()
        self._puppet_arm.move_to_neutral()

    def puppet(self):
        """

        """
        self.set_neutral()
        rate = rospy.Rate(100);
        start = rospy.Time.now()

        control_joint_names = self._control_arm.joint_names()
        puppet_joint_names = self._puppet_arm.joint_names()

        done = False
        print("Puppeting. Press any key to stop...")
        while not done and not rospy.is_shutdown():
            if iodevices.getch():
                done = True
            else:
                elapsed = rospy.Time.now() - start
                cmd = {}
                for idx, name in enumerate(puppet_joint_names):
                    v = self._control_arm.joint_velocity(control_joint_names[idx])
                    if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
                        v = -v
                    cmd[name] = v * self._amp
                self._puppet_arm.set_joint_velocities(cmd)
                rate.sleep()

        rate = rospy.Rate(100);
        if not rospy.is_shutdown():
            for i in range(100):
                if rospy.is_shutdown():
                    return False
                self._control_arm.set_joint_position_mode()
                self._puppet_arm.set_joint_position_mode()
                rate.sleep()
            #return to normal
            self.set_neutral()
            return True

if __name__ == '__main__':
    max_gain = 3.0
    min_gain = 0.1

    parser = argparse.ArgumentParser()
    parser.add_argument("limb", help="specify the limb to puppet: left or right")
    parser.add_argument("-a", "--amplification", dest="amplification", type=float,
        default=1.0, help=("amplification to apply to the puppeted arm [%g, %g]" % (min_gain, max_gain)))
    args, unknown = parser.parse_known_args()
    if (args.amplification < min_gain or max_gain < args.amplification):
        print("Exiting: Amplification must be between: [%g, %g]" % (min_gain, max_gain))
        sys.exit(1)

    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_velocity_puppet", anonymous=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    puppeteer = Puppeteer(args.limb, args.amplification)
    puppeteer.puppet()

    print("Disabling robot... ")
    rs.disable()
    print("done.")
