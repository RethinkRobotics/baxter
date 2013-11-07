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

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface


class Puppeteer(object):

    def __init__(self, limb, amplification=1.0):
        """
        @param limb - the control arm used to puppet the other
        @param amplification - factor by which to amplify the arm movement

        Puppets one arm with the other.
        """
        puppet_arm = {"left": "right", "right": "left"}
        self._control_limb = limb
        self._puppet_limb = puppet_arm[limb]
        self._control_arm = baxter_interface.limb.Limb(self._control_limb)
        self._puppet_arm = baxter_interface.limb.Limb(self._puppet_limb)
        self._amp = amplification

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._control_arm.exit_control_mode()
            self._puppet_arm.exit_control_mode()
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._control_arm.move_to_neutral()
        self._puppet_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def puppet(self):
        """

        """
        self.set_neutral()
        rate = rospy.Rate(100)

        control_joint_names = self._control_arm.joint_names()
        puppet_joint_names = self._puppet_arm.joint_names()

        print ("Puppeting:\n"
              "  Grab %s cuff and move arm.\n"
              "  Press Ctrl-C to stop...") % (self._control_limb,)
        while not rospy.is_shutdown():
            cmd = {}
            for idx, name in enumerate(puppet_joint_names):
                v = self._control_arm.joint_velocity(
                    control_joint_names[idx])
                if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
                    v = -v
                cmd[name] = v * self._amp
            self._puppet_arm.set_joint_velocities(cmd)
            rate.sleep()


def main():
    max_gain = 3.0
    min_gain = 0.1

    parser = argparse.ArgumentParser()
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-l", "--limb", required=True, choices=['left', 'right'],
        help="specify the puppeteer limb (the control limb)"
    )
    parser.add_argument(
        "-a", "--amplification", type=float, default=1.0,
        help=("amplification to apply to the puppeted arm [%g, %g]"
              % (min_gain, max_gain))
    )
    args = parser.parse_args(rospy.myargv()[1:])
    if (args.amplification < min_gain or max_gain < args.amplification):
        print("Exiting: Amplification must be between: [%g, %g]" %
              (min_gain, max_gain))
        return 1

    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_velocity_puppet")

    puppeteer = Puppeteer(args.limb, args.amplification)
    rospy.on_shutdown(puppeteer.clean_shutdown)
    puppeteer.puppet()

    print("Done.")
    return 0

if __name__ == '__main__':
    sys.exit(main())
