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

"""
Baxter RSDK Joint Trajectory Controller
    Unlike other robots running ROS, this is not a Motor Controller plugin,
    but a regular node using the SDK interface.
"""
import sys
import argparse

import roslib
roslib.load_manifest('baxter_interface')
import rospy

from dynamic_reconfigure.server import Server
from baxter_interface.cfg import (
    JointTrajectoryActionServerConfig
)

import iodevices
import dataflow
import baxter_interface
from joint_trajectory import (
    JointTrajectoryActionServer,
)

def usage(argv):
    print "usage: " + argv[0] + " <namespace>"

def main(limb, rate):
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_trajectory_controller%s" % ("" if limb == 'both' else "_" + limb,))
    print("Initializing trajectory interface...")


    dynamic_cfg_srv = Server(JointTrajectoryActionServerConfig,
                                 lambda config,level: config)
    if limb == 'both':
        JointTrajectoryActionServer('right', dynamic_cfg_srv, rate)
        JointTrajectoryActionServer('left', dynamic_cfg_srv, rate)
    else:
        JointTrajectoryActionServer(limb, rate, )
    print("Running. Ctrl-c to quit")
    rospy.spin()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-l", "--limb", dest="limb", default="both", help="trajectory controller limb [both | left | right]")
    parser.add_argument("-r", "--rate", dest="rate", default=100.0, type=float, help="trajectory control rate (Hz)")
    args = parser.parse_args()
    main(args.limb, args.rate)