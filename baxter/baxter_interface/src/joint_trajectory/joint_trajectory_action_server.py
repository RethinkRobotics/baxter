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
Baxter RSDK Joint Trajectory Action Server
"""
import math

import roslib
roslib.load_manifest('baxter_interface')
import rospy
import actionlib

from std_msgs.msg import (
    UInt16,
)
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


class JointTrajectoryActionServer(object):

    def __init__(self, limb, rate=100.0):
        self._ns = '/sdk/robot/limb/' + limb + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_fjta,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()
        self._limb = baxter_interface.Limb(limb)

        # Controller params from arguments and param server
        self._control_rate = rate  # Hz
        self._control_joints = []
        self._pid_gains = {'kp': {}, 'ki': {}, 'kd': {}}
        self._goal_time = 0.0
        self._goal_error = {}
        self._error_threshold = {}

        # Create our PID controllers
        self._pid = {}
        for joint in self._limb.joint_names():
            self._pid[joint] = control.PID()

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate', UInt16)
        self._pub_rate.publish(self._control_rate)

    def _get_trajectory_parameters(self, joint_names):
        self._control_joints = rospy.get_param(
            self._ns + '/joints', joint_names)
        self._goal_time = rospy.get_param(
            self._ns + '/constraints/goal_time', 0.0)
        for joint in self._control_joints:
            self._goal_error[joint] = rospy.get_param(
                self._ns + '/constraints/' + joint + '/goal', -1.0)
            self._error_threshold[joint] = rospy.get_param(
                self._ns + '/constraints/' + joint + '/trajectory', -1.0)
            self._pid_gains['kp'][joint] = rospy.get_param(
                self._ns + '/constraints/' + joint + '/kp', 2.0)
            self._pid_gains['ki'][joint] = rospy.get_param(
                self._ns + '/constraints/' + joint + '/ki', 0.0)
            self._pid_gains['kd'][joint] = rospy.get_param(
                self._ns + '/constraints/' + joint + '/kd', 0.0)

    def _discretize_trajectory(self, trajectory, discretization):
        """
        @param trajectory trajectory_msgs.msg.JointTrajectory - input trajectory
        @param discretization float - control rate (Hz)

        Creates a linearly interpolated discretized trajectory across each joint.
        """
        trajectory_out = JointTrajectory()
        trajectory_out.joint_names = trajectory.joint_names
        # Add a new point representing our current position
        # if time from start for the first position is greater than 0
        if trajectory.points[0].time_from_start > rospy.Duration(0.0):
            start_point = JointTrajectoryPoint()
            start_position = self._limb.joint_angles()
            for joint in self._control_joints:
                start_point.positions.append(start_position(joint))
            start_point.time_from_start = rospy.Duration(0.0)
            trajectory.points.insert(0, start_point)
        # Step over our original trajectory, inserting discretized points
        # in time and position for the specified control rate
        for i in xrange(1, len(trajectory.points)):
            # Determine the time step intervals based on the time
            # to achieve the next point and the control rate
            move_interval = int(
                ((trajectory.points[i].time_from_start - trajectory.points[i - 1].time_from_start).to_sec()) / (1.0 / discretization))
            # Create new points for every control cycle to achieve a linear
            # trajectory across each joint to arrive at the goal point at
            # exactly the specified time
            for j in xrange(move_interval):
                point = JointTrajectoryPoint()
                diff = []
                j = j + 1.0
                for k in xrange(len(trajectory.points[i].positions)):
                    start = trajectory.points[i - 1].positions[k]
                    end = trajectory.points[i].positions[k]
                    diff.append(end - start)
                    point.positions.append(
                        start + diff[k] * (j / move_interval))
                point.time_from_start = rospy.Duration(
                    trajectory.points[i - 1].time_from_start.to_sec() + j * (1.0 / discretization))
                trajectory_out.points.append(point)
        return trajectory_out

    def _get_current_error(self, joint_names, set_point):
        current = [self._limb.joint_angle(joint) for joint in joint_names]
        return zip(joint_names, [(tgt - cur) for tgt, cur in zip(set_point, current)])

    def _command_velocities(self, joint_names, point):
        velocities = []
        if self._server.is_preempt_requested():
            self._command_stop(joint_names)
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            return False
        deltas = self._get_current_error(joint_names, point.positions)
        for delta in deltas:
            if delta[1] >= self._error_threshold[delta[0]] and self._error_threshold[delta[0]] >= 0.0:
                self._command_stop(joint_names)
                rospy.logerr("%s: Trajectory Aborted - Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._server.set_aborted()
                return False
            velocities.append(self._pid[delta[0]].compute_output(delta[1]))
        cmd = dict(zip(joint_names, velocities))
        self._limb.set_joint_velocities(cmd)
        return True

    def _command_stop(self, joint_names):
        velocities = []
        for joint in joint_names:
            velocities.append(0.0)
            cmd = dict(zip(joint, velocities))
            self._limb.set_joint_velocities(cmd)

    def _on_fjta(self, goal):
        self._get_trajectory_parameters(goal.trajectory.joint_names)
        # Verify control_joints and set PID gains
        for joint in goal.trajectory.joint_names:
            if joint in self._limb.joint_names() and joint in self._control_joints:
                self._pid[joint].set_kp(self._pid_gains['kp'][joint])
                self._pid[joint].set_ki(self._pid_gains['ki'][joint])
                self._pid[joint].set_kd(self._pid_gains['kd'][joint])
                self._pid[joint].initialize()
            else:
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, joint,))
                self._server.set_aborted()
                return
        # Create a new discretized joint trajectory
        trajectory = JointTrajectory()
        trajectory = self._discretize_trajectory(
            goal.trajectory, self._control_rate)
        # Wait for the specified execution time, if not provided use now
        start_time = trajectory.header.stamp.to_sec()
        if start_time == 0:
            start_time = rospy.get_time()
        dataflow.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float("inf")
        )
        control_rate = rospy.Rate(self._control_rate)
        success = True
        # Execute the trajectory
        for point in trajectory.points:
            arrive_at = point.time_from_start.to_sec() + start_time
            while rospy.get_time() < arrive_at and success:
                success = self._command_velocities(trajectory.joint_names, point)
                if not success:
                    return
                control_rate.sleep()

        # Keep trying to meet goal until goal_time constraint expired
        while ((rospy.get_time() < start_time + trajectory.points[-1].time_from_start.to_sec() + self._goal_time) and any((self._goal_error[error[0]] > 0 and self._goal_error[error[0]] < math.fabs(error[1]) for error in self._get_current_error(trajectory.joint_names, trajectory.points[-1].positions)))):
            success = self._command_velocities(
                trajectory.joint_names,
                trajectory.points[-1])
            if not success:
                return
            control_rate.sleep()

        # Verify goal constraint
        if outside_goal_tolerance:
            self._command_stop(trajectory.joint_names)
            rospy.logerr("%s: Trajectory Failed - Exceeded Goal Threshold Error" %
                         (self._action_name,))
            self._server.set_aborted()
        else:
            self._command_stop(trajectory.joint_names)
            self._server.set_succeeded()
