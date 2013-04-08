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
Baxter RSDK Follow Joint Trajectory Action Server
"""
import roslib
roslib.load_manifest('baxter_interface')
import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
import dataflow
import baxter_interface

class FJTAS(object):
    def __init__(self, limb):
        limbns = {'left':'l_arm_controller', 'right':'r_arm_controller'}
        self._server = actionlib.SimpleActionServer(
            limbns[limb] + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._on_fjta,
            auto_start=False)
        self._server.start()
        self._limb = baxter_interface.Limb(limb)

    def _on_fjta(self, goal):
        trajectory = goal.trajectory
        start_time = trajectory.header.stamp.to_sec()
        dataflow.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float("inf")
        )
        rate = rospy.Rate(1000)
        for point in trajectory.points:
            arrive_at = point.time_from_start.to_sec()
            time_left = arrive_at - (rospy.get_time() - start_time)
            while time_left > 0:
                try:
                    current = [self._limb.joint_angle(j) for j in trajectory.joint_names]
                except KeyError as e:
                    rospy.logerr("unable to execute trajectory: ", e.strerror)
                    return
                deltas = [(tgt-cur) for tgt,cur in zip(point.positions, current)]
                velocities = [d / time_left for d in deltas]
                cmd = dict(zip(trajectory.joint_names, velocities))
                self._limb.set_velocities(cmd)
                rate.sleep()
                time_left = arrive_at - (rospy.get_time() - start_time)
        self._server.set_succeeded()
