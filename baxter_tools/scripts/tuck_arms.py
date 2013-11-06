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
import argparse
from copy import deepcopy

import rospy

from std_msgs.msg import (
    Empty,
    Bool,
)

import baxter_interface

from baxter_core_msgs.msg import (
    CollisionAvoidanceState,
)


class Tuck(object):
    def __init__(self, tuck_cmd):
        self._done = False
        self._limbs = ('left', 'right')
        self._arms = {
            'left': baxter_interface.Limb('left'),
            'right': baxter_interface.Limb('right'),
            }
        self._tuck = tuck_cmd
        self._tuck_rate = rospy.Rate(20.0)  # Hz
        self._tuck_threshold = 0.2  # radians
        self._peak_angle = -1.6  # radians
        self._arm_state = {
                           'tuck': {'left': 'none', 'right': 'none'},
                           'collide': {'left': False, 'right': False},
                           'flipped': {'left': False, 'right': False}
                          }
        self._joint_moves = {
            'tuck': {
                     'left':  [-1.0, -2.07,  3.0, 2.55,  0.0, 0.01,  0.0],
                     'right':  [1.0, -2.07, -3.0, 2.55, -0.0, 0.01,  0.0]
                     },
            'untuck': {
                       'left':  [-0.08, -1.0, -1.19, 1.94,  0.67, 1.03, -0.50],
                       'right':  [0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50]
                       }
            }
        self._collide_lsub = rospy.Subscriber(
                             'robot/limb/left/collision_avoidance_state',
                             CollisionAvoidanceState,
                             self._update_collision, 'left')
        self._collide_rsub = rospy.Subscriber(
                             'robot/limb/right/collision_avoidance_state',
                             CollisionAvoidanceState,
                             self._update_collision, 'right')
        self._disable_pub = {
            'left': rospy.Publisher(
                 'robot/limb/left/suppress_collision_avoidance',
                 Empty),
            'right': rospy.Publisher(
                 'robot/limb/right/suppress_collision_avoidance',
                 Empty)
        }
        self._rs = baxter_interface.RobotEnable()
        self._enable_pub = rospy.Publisher('robot/set_super_enable', Bool)

    def _update_collision(self, data, limb):
        self._arm_state['collide'][limb] = len(data.collision_object) > 0
        self._check_arm_state()

    def _check_arm_state(self):
        """
        Check for goals and behind collision field.

        If s1 joint is over the peak, collision will need to be disabled
        to get the arm around the head-arm collision force-field.
        """
        diff_check = lambda a, b: abs(a - b) <= self._tuck_threshold
        for limb in self._limbs:
            angles = [self._arms[limb].joint_angle(joint)
                      for joint in self._arms[limb].joint_names()]

            # Check if in a goal position
            untuck_goal = map(diff_check, angles,
                              self._joint_moves['untuck'][limb])
            tuck_goal = map(diff_check, angles[0:2],
                            self._joint_moves['tuck'][limb][0:2])
            if all(untuck_goal):
                self._arm_state['tuck'][limb] = 'untuck'
            elif all(tuck_goal):
                self._arm_state['tuck'][limb] = 'tuck'
            else:
                self._arm_state['tuck'][limb] = 'none'

            # Check if shoulder is flipped over peak
            self._arm_state['flipped'][limb] = (
                self._arms[limb].joint_angle(limb + '_s1') <= self._peak_angle)

    def _prepare_to_tuck(self):
        # If arms are in "tucked" state, disable collision avoidance
        # before enabling robot, to avoid arm jerking from "force-field".
        head = baxter_interface.Head()
        start_disabled = not self._rs.state().enabled
        at_goal = lambda: (abs(head.pan()) <=
                        baxter_interface.settings.HEAD_PAN_ANGLE_TOLERANCE)

        rospy.loginfo("Moving head to neutral position")
        while not at_goal() and not rospy.is_shutdown():
            if start_disabled:
                [pub.publish(Empty()) for pub in self._disable_pub.values()]
            if not self._rs.state().enabled:
                self._enable_pub.publish(True)
            head.set_pan(0.0, 50.0, timeout=0)
            self._tuck_rate.sleep()

        if start_disabled:
            while self._rs.state().enabled == True and not rospy.is_shutdown():
                [pub.publish(Empty()) for pub in self._disable_pub.values()]
                self._enable_pub.publish(False)
                self._tuck_rate.sleep()

    def _move_to(self, tuck, disabled):
        if any(disabled.values()):
            [pub.publish(Empty()) for pub in self._disable_pub.values()]
        while (any(self._arm_state['tuck'][limb] != goal
                   for limb, goal in tuck.viewitems())
               and not rospy.is_shutdown()):
            if self._rs.state().enabled == False:
                self._enable_pub.publish(True)
            for limb in self._limbs:
                if disabled[limb]:
                    self._disable_pub[limb].publish(Empty())
                if limb in tuck:
                    self._arms[limb].set_joint_positions(dict(zip(
                                      self._arms[limb].joint_names(),
                                      self._joint_moves[tuck[limb]][limb])))
            self._check_arm_state()
            self._tuck_rate.sleep()

        if any(self._arm_state['collide'].values()):
            self._rs.disable()
        return

    def supervised_tuck(self):
        # Update our starting state to check if arms are tucked
        self._prepare_to_tuck()
        self._check_arm_state()
        # Tuck Arms
        if self._tuck == True:
            # If arms are already tucked, report this to user and exit.
            if all(self._arm_state['tuck'][limb] == 'tuck'
                   for limb in self._limbs):
                rospy.loginfo("Tucking: Arms already in 'Tucked' position.")
                self._done = True
                return
            else:
                rospy.loginfo("Tucking: One or more arms not Tucked.")
                any_flipped = not all(self._arm_state['flipped'].values())
                if any_flipped:
                    rospy.loginfo(
                        "Moving to neutral start position with collision %s.",
                        "on" if any_flipped else "off")
                # Move to neutral pose before tucking arms to avoid damage
                self._check_arm_state()
                actions = dict()
                disabled = {'left': True, 'right': True}
                for limb in self._limbs:
                    if not self._arm_state['flipped'][limb]:
                        actions[limb] = 'untuck'
                        disabled[limb] = False
                self._move_to(actions, disabled)

                # Disable collision and Tuck Arms
                rospy.loginfo("Tucking: Tucking with collision avoidance off.")
                actions = {'left': 'tuck', 'right': 'tuck'}
                disabled = {'left': True, 'right': True}
                self._move_to(actions, disabled)
                self._done = True
                return

        # Untuck Arms
        else:
            # If arms are tucked disable collision and untuck arms
            if any(self._arm_state['flipped'].values()):
                rospy.loginfo("Untucking: One or more arms Tucked;"
                              " Disabling Collision Avoidance and untucking.")
                self._check_arm_state()
                suppress = deepcopy(self._arm_state['flipped'])
                actions = {'left': 'untuck', 'right': 'untuck'}
                self._move_to(actions, suppress)
                self._done = True
                return
            # If arms already untucked, move to neutral location
            else:
                rospy.loginfo("Untucking: Arms already Untucked;"
                              " Moving to neutral position.")
                self._check_arm_state()
                suppress = deepcopy(self._arm_state['flipped'])
                actions = {'left': 'untuck', 'right': 'untuck'}
                self._move_to(actions, suppress)
                self._done = True
                return

    def clean_shutdown(self):
        """Handles ROS shutdown (Ctrl-C) safely."""
        if not self._done:
            rospy.logwarn('Aborting: Shutting down safely...')
        if any(self._arm_state['collide'].values()):
            while self._rs.state().enabled != False:
                [pub.publish(Empty()) for pub in self._disable_pub.values()]
                self._enable_pub.publish(False)
                self._tuck_rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    tuck_group = parser.add_mutually_exclusive_group(required=True)
    tuck_group.add_argument("-t", "--tuck", dest="tuck",
        action='store_true', default=False, help="tuck arms")
    tuck_group.add_argument("-u", "--untuck", dest="untuck",
        action='store_true', default=False, help="untuck arms")
    args = parser.parse_args(rospy.myargv()[1:])
    tuck = args.tuck

    rospy.loginfo("Initializing node... ")
    rospy.init_node("tuck_arms")
    rospy.loginfo("%sucking arms" % ("T" if tuck else "Unt",))
    tucker = Tuck(tuck)
    rospy.on_shutdown(tucker.clean_shutdown)
    tucker.supervised_tuck()
    rospy.loginfo("Finished tuck")

if __name__ == "__main__":
    main()
