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
roslib.load_manifest('head_control')
import rospy
import baxter_interface

class Wobbler():

    def __init__(self):
        """
        'Wobbles' the head

        """
        self._done = False
        signal.signal(signal.SIGINT, self._handle_ctrl_c)
        self._head = baxter_interface.Head()

    def _handle_ctrl_c(self, signum, frame):
       print("stopping...")
       self._done = True

    def set_neutral(self):
        """
        Sets the head back into a neutral pose

        """
        self._head.set_pan(0.0)

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling

        """
        self._head.command_nod()
        rate = rospy.Rate(1);
        start = rospy.get_time()
        while (rospy.get_time() - start < 5.0):
            angle = random.uniform(-1.5, 1.5)
            self._head.set_pan(angle)
            rate.sleep();

        #return to normal
        self.set_neutral()

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
