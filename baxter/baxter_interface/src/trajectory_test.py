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
Baxter RSDK Joint Trajectory Controller Test
"""
import sys
from copy import copy

import roslib
roslib.load_manifest('baxter_interface')
import rospy
import actionlib

import iodevices
import dataflow
import baxter_interface
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

def usage(argv):
    print "usage: " + argv[0] + " <namespace>"

class Trajectory(object):
    def __init__(self, limb):
        limbns = {'left':'l_arm_controller', 'right':'r_arm_controller'}
        self._client = actionlib.SimpleActionClient(
            limbns[limb] + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._client.wait_for_server()
        self.clear()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        pass

    def wait(self):
        self._client.wait_for_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = ['s1', 's2', 'e0', 'e1', 'w0', 'w1', 'w2']

def main(ns):
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_trajectory_controller_test")
    print("Running. Ctrl-c to quit")
    traj = Trajectory(ns)
    p1 = [0.20, -0.66, 1.15, 1.09, 2.53, -1.56, 2.34]
    traj.add_point(p1, 2.0)
    traj.add_point([x * 0.9 for x in p1], 4.0)
    traj.add_point([x * 1.1 for x in p1], 7.0)
    traj.start()
    traj.wait()
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        ns = sys.argv[1]
        main(ns)
    else:
        usage(sys.argv)
