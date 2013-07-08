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
Baxter RSDK Joint Trajectory Example: file playback
Plays back joint positions honoring timestamps recorded
Via joint_position example - joint_position recorder.py <filename>
"""


import sys
from copy import copy
import operator
import argparse

import roslib
roslib.load_manifest('joint_trajectory')
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

class Trajectory(object):
    def __init__(self):
        l_server = '/sdk/robot/limb/left/follow_joint_trajectory'
        r_server = '/sdk/robot/limb/right/follow_joint_trajectory'
        self._left_client = actionlib.SimpleActionClient(
            l_server,
            FollowJointTrajectoryAction,
        )
        self._right_client = actionlib.SimpleActionClient(
            r_server,
            FollowJointTrajectoryAction,
        )

        l_server_up = self._left_client.wait_for_server(rospy.Duration(1.0))
        r_server_up = self._right_client.wait_for_server(rospy.Duration(1.0))
        if not l_server_up or not r_server_up:
            msg = "Action server not available. Verify trajectory controller."
            rospy.logerr(msg)
            sys.exit(0)
        self._l_goal = FollowJointTrajectoryGoal()
        self._r_goal = FollowJointTrajectoryGoal()

        # To get current angles for start move
        self._l_arm = baxter_interface.Limb('left')
        self._r_arm = baxter_interface.Limb('right')

    def clean_line(self, line, joint_names):
        """ Cleans a single line of recorded joint positions
        @param line - the line described in a list to process
        @param joint_names - joint name keys
        """
        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None
        #convert the line of strings to a float or None
        line = [try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(joint_names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        return (command, line,)

    def parse_file(self, filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
        joint_names = lines[0].rstrip().split(',')
        for name in joint_names:
            if name[:-2] == 'left_' and name != 'left_gripper':
                self._l_goal.trajectory.joint_names.append(name)
            if name[:-2] == 'right_' and name != 'right_gripper':
                self._r_goal.trajectory.joint_names.append(name)

        def find_start_offset(pos):
            cur, cmd = ([] for i in range(2))
            for joint in joint_names:
                if 'left' == joint[:-3]:
                    cmd.append(pos[joint])
                    cur.append(self._l_arm.joint_angle(joint))
                elif 'right' == joint[:-3]:
                    cmd.append(pos[joint])
                    cur.append(self._r_arm.joint_angle(joint))
            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            default_velocities = [0.25] * 14
            offset = max(map(operator.div, diffs, default_velocities))
            return offset - float(lines[1].rstrip().split(',')[0])

        for idx, values in enumerate(lines[1:]):
            cmd, values = self.clean_line(values, joint_names)
            #find allowable time offset for move to start position
            if idx == 0:
                start_offset = find_start_offset(cmd)
            #add a point for this set of commands recorded time
            cur_cmd = [cmd[jnt] for jnt in self._l_goal.trajectory.joint_names]
            self.add_point(cur_cmd, 'left', values[0] + start_offset)
            cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
            self.add_point(cur_cmd, 'right', values[0] + start_offset)

    def add_point(self, positions, side, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        if side == 'left':
            self._l_goal.trajectory.points.append(point)
        elif side == 'right':
            self._r_goal.trajectory.points.append(point)

    def start(self):
        self._left_client.send_goal(self._l_goal)
        self._right_client.send_goal(self._r_goal)

    def stop(self):
        self._left_client.cancel_goal()
        self._right_client.cancel_goal()

    def wait(self):
        last_time = self._r_goal.trajectory.points[-1].time_from_start.to_sec()
        timeout = rospy.Duration(2.0 * last_time)

        l_finish = self._left_client.wait_for_result(timeout)
        r_finish = self._right_client.wait_for_result(timeout)

        if l_finish and r_finish:
            return True
        else:
            print("Trajectory action did not finish before timeout.")
            return False

def main(file, loops):
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_trajectory_file_playback")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = Trajectory()
    traj.parse_file(file)
    result = True
    loop_cnt = 1
    loopstr = str(loops)
    if loops == 0:
        loops = float('inf')
        loopstr = "forever"
    while result == True and loop_cnt <= loops:
        print("Playback loop %d of %s" % (loop_cnt, loopstr,))
        traj.start()
        result = traj.wait()
        loop_cnt = loop_cnt + 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', dest='file', required=True,
        help="input file")
    parser.add_argument('-l', '--loops', dest='loops', type=int, default=1,
        help="number of playback loops. 0=infinite.")
    args = parser.parse_args()
    main(args.file, args.loops)