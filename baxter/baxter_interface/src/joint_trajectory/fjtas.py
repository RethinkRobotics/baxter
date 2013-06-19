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

from std_msgs.msg import (
    UInt16,)
from control_msgs.msg import (
    FollowJointTrajectoryAction,
)
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

import dataflow
import control
import baxter_interface

class FJTAS(object):
    def __init__(self, limb, rate, kp, ki, kd):
        ns = '/sdk/robot/limb/' + limb + '/'
        self._server = actionlib.SimpleActionServer(
            ns + 'follow_joint_trajectory',
            FollowJointTrajectoryAction,
            execute_cb=self._on_fjta,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()
        self._limb = baxter_interface.Limb(limb)

        self._control_rate = rate  # Hz

        # set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        self._pub_rate.publish(self._control_rate)

        # set PID controller gains
        self._pid = control.PID()
        self._pid.set_kp(kp)
        self._pid.set_ki(ki)
        self._pid.set_kd(kd)

    def _discretize_trajectory(self, trajectory, discretization):
        """
        @param trajectory trajectory_msgs.msg.JointTrajectory - input trajectory
        @param discretization float - control rate (Hz)

        Creates a linear interpolated discretized trajectory across each joint.
        """
        trajectory_out = JointTrajectory()
        trajectory_out.joint_names = trajectory.joint_names
        # Add a new point representing our current position
        start_point = JointTrajectoryPoint()
        for joint in self._limb.joint_names():
            start_point.positions.append(self._limb.joint_angle(joint))
        start_point.time_from_start = rospy.Duration(0.0)
        trajectory.points.insert(0, start_point)

        # Step over our original trajectory, inserting discretized points in
        # in time and position for the specified control rate
        for i in xrange(1, len(trajectory.points)):
            # Determine the time step intervals based on the time
            # to achieve the next point and the control rate
            move_interval = int(((trajectory.points[i].time_from_start - \
                trajectory.points[i - 1].time_from_start).to_sec()) \
                / (1.0 / discretization))
            # Create new points for every control cycle to achieve a linear
            # trajectory across each joint to arrive at the goal point at
            # exactly the specified time
            for k in xrange(move_interval):
                point = JointTrajectoryPoint()
                diff = []
                k = float(k + 1.0)
                for p in xrange(len(trajectory.points[i].positions)):
                    start = trajectory.points[i - 1].positions[p]
                    end = trajectory.points[i].positions[p]
                    diff.append(end - start)
                    point.positions.append(start + diff[p] * (k / move_interval))
                point.time_from_start = rospy.Duration(trajectory.points[i - 1].time_from_start.to_sec() + k * (1.0 / discretization))
                trajectory_out.points.append(point)
        return trajectory_out

    def _on_fjta(self, goal):
        self._pid.initialize()
        # Create a new discretized joint trajectory
        trajectory = JointTrajectory()
        trajectory = self._discretize_trajectory(goal.trajectory, self._control_rate)
        # Wait for the specified execution time, if not provided use now
        start_time = trajectory.header.stamp.to_sec()
        if start_time == 0:
            start_time = rospy.get_time()
        dataflow.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float("inf")
        )
        control_rate = rospy.Rate(self._control_rate)
        # Execute the trajectory
        for point in trajectory.points:
            arrive_at = point.time_from_start.to_sec()
            time_left = arrive_at - (rospy.get_time() - start_time)
            while time_left > 0:
                if self._server.is_preempt_requested():
                    velocities = []
                    for i in xrange(len(trajectory.joint_names)):
                        velocities.append(0.0)
                        cmd = dict(zip(trajectory.joint_names, velocities))
                        self._limb.set_joint_velocities(cmd)
                    rospy.loginfo("%s: Trajectory Preempted" % self._action_name)
                    self._server.set_preempted()
                    return
                try:
                    current = [self._limb.joint_angle(j) for j in trajectory.joint_names]
                except KeyError as e:
                    rospy.logerr("%s: Trajectory Provided Invalid Joint Name %s" % (self._action_name, e))
                    self._server.set_aborted()
                    return
                deltas = [(tgt - cur) for tgt, cur in zip(point.positions, current)]
                velocities = []
                for delta in deltas:
                    velocities.append(self._pid.compute_output(delta))
                cmd = dict(zip(trajectory.joint_names, velocities))
                self._limb.set_joint_velocities(cmd)
                control_rate.sleep()
                time_left = arrive_at - (rospy.get_time() - start_time)

        self._server.set_succeeded()
