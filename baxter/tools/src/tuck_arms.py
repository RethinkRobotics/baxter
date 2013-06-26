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
Tool to tuck/untuck Baxter's arms to/from the shipping pose
"""
import math
import argparse
import threading

import roslib
roslib.load_manifest('tools')
import rospy
from std_msgs.msg import (
    Empty,
)
import baxter_interface
import dataflow

class Tuck(object):
    def __init__(self, tuck_cmd):
        self._limbs = ('left', 'right')
        self._arms = {
            'left': baxter_interface.Limb('left'),
            'right': baxter_interface.Limb('right'),
            }
        self._tuck = tuck_cmd
        self._tuck_rate = rospy.Rate(20.0) # Hz
        self._tuck_threshold = 0.6 # Radians
        self._tuck_state = {'left': False, 'right': False}
        self._joint_moves = {
            'tuck': {
                     'left': [ -1.0, -2.07,   3.0, 2.55,  0.0, 0.01,   0.0],
                     'right': [ 1.0, -2.07, -3.0, 2.55, -0.0, 0.01,  0.0]
                     },
            'untuck': {
                       'left': [-0.08,  -1.0, -1.19, 1.94, 0.67, 1.03, -0.50],
                       'right': [0.08,  -1.0, 1.19, 1.94, -0.67, 1.03, 0.50]
                       }
        }
        self._disable_pub = {
            'left': rospy.Publisher(
                 '/robot/limb/left/CollisionAvoidance/suppress_body_avoidance',
                 Empty),
            'right': rospy.Publisher(
                 '/robot/limb/right/CollisionAvoidance/suppress_body_avoidance',
                 Empty)
        }
        self._rs = baxter_interface.RobotEnable()
        self._supervised_tuck()

    def _check_tucked(self, goal):
        for limb in self._limbs:
            diff = 0.0
            for joint_idx, joint in enumerate(self._arms[limb].joint_names()):
                if joint[-2:] in ('s0', 's1', 'e0') or goal == 'untuck':
                    diff = diff + abs(self._joint_moves[goal][limb][joint_idx] - self._arms[limb].joint_angle(joint))
            self._tuck_state[limb] = diff < self._tuck_threshold

    def _reset_tucked_state(self):
        for limb in self._limbs:
            self._tuck_state[limb] = False

    def _move_to(self, tuck, disabled):
        self._rs.enable()
        while any(self._tuck_state[limb] != True for limb in self._limbs) and not rospy.is_shutdown():
            for limb in self._limbs:
                if disabled:
                    self._disable_pub[limb].publish(Empty())
                self._arms[limb].set_joint_positions(dict(zip(self._arms[limb].joint_names(),self._joint_moves[tuck][limb])))
            self._check_tucked(tuck)
            self._tuck_rate.sleep()
        self._reset_tucked_state()
        if (tuck == 'tuck'):
            self._rs.disable()
        return

    def _supervised_tuck(self):
        # Check our starting state to see if arms are tucked
        self._check_tucked('tuck')
        # Tuck Arms
        if self._tuck == True:
            # If arms are already tucked, report this to user and exit.
            if all(self._tuck_state[limb] == True for limb in self._limbs):
                rospy.loginfo("Arms already in 'Tucked' position.")
                return
            else:
                any_tucked = any(self._tuck_state[limb] == True for limb in self._limbs)
                # Move to neutral location before tucking arms to avoid damaging the robot
                self._move_to('untuck', any_tucked)
                # Disable collision and Tuck Arms
                self._move_to('tuck', True)
                return
        # Untuck Arms
        else:
            # If arms are tucked disable collision and untuck arms.
            if any(self._tuck_state[limb] == True for limb in self._limbs):
                self._reset_tucked_state()
                # Disable collision and untuck Arms
                self._move_to('untuck', True)
                return
            # If arms already untucked, move to neutral location
            else:
                self._move_to('untuck', False)
                return

def main(tuck):
    print("Initializing node... ")
    rospy.init_node("tuck_arms")
    print("Moving head to neutral position")
    head = baxter_interface.Head()
    head.set_pan(0.0, 5.0)
    print("%sucking arms" % ("T" if tuck else "Unt",))
    Tuck(tuck)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    tuck_group = parser.add_mutually_exclusive_group(required=True)
    tuck_group.add_argument("-t", "--tuck", dest="tuck", \
        action='store_true', default=False, help="tuck arms")
    tuck_group.add_argument("-u", "--untuck", dest="untuck", \
        action='store_true', default=False, help="untuck arms")
    args = parser.parse_args()
    main(args.tuck)
