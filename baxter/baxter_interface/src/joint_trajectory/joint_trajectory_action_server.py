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
import operator
import bisect

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
    def __init__(self, limb, parameters, rate=100.0):
        self._param = parameters
        self._ns = '/sdk/robot/limb/' + limb + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()
        self._server.start()
        self._limb = baxter_interface.Limb(limb)

        # Controller parameters from arguments and dynamic reconfigure
        self._control_rate = rate  # Hz
        self._control_joints = []
        self._pid_gains = {'kp': {}, 'ki': {}, 'kd': {}}
        self._goal_time = 0.0
        self._goal_error = {}
        self._error_threshold = {}
        self._dflt_vel = {}

        # Create our PID controllers
        self._pid = {}
        for joint in self._limb.joint_names():
            self._pid[joint] = control.PID()

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate', UInt16)
        self._pub_rate.publish(self._control_rate)

    def _get_trajectory_parameters(self, joint_names):
        for jnt in joint_names:
            if not jnt in self._limb.joint_names():
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, jnt,))
                self._server.set_aborted()
                return False

            self._pid[jnt].set_kp(self._param.config[jnt + '_kp'])
            self._pid[jnt].set_ki(self._param.config[jnt + '_ki'])
            self._pid[jnt].set_kd(self._param.config[jnt + '_kd'])
            self._goal_error[jnt] = self._param.config[jnt + '_goal']
            self._error_threshold[jnt] = self._param.config[jnt + '_trajectory']
            self._dflt_vel[jnt] = self._param.config[jnt + '_default_velocity']
            self._pid[jnt].initialize()
        return True

    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position(joint_names)
        error = map(operator.sub, set_point, current)
        return zip(joint_names, error)

    def _command_stop(self, joint_names):
        velocities = [0.0] * len(joint_names)
        cmd = dict(zip(joint_names, velocities))
        self._limb.set_joint_velocities(cmd)

    def _command_velocities(self, joint_names, positions):
        velocities = []
        if self._server.is_preempt_requested():
            self._command_stop(joint_names)
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            return False
        deltas = self._get_current_error(joint_names, positions)
        for delta in deltas:
            if (delta[1] >= self._error_threshold[delta[0]]
                and self._error_threshold[delta[0]] >= 0.0):
                self._command_stop(joint_names)
                rospy.logerr(
                             "%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._server.set_aborted()
                return False
            velocities.append(self._pid[delta[0]].compute_output(delta[1]))
        cmd = dict(zip(joint_names, velocities))
        self._limb.set_joint_velocities(cmd)
        return True

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        # Load parameters for trajectory
        if not self._get_trajectory_parameters(joint_names):
            self._server.set_aborted()
            return

        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)

        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return

        # If all time_from_start are zero,
        # interject these based on default velocities
        if all(pt.time_from_start.to_sec() == 0.0 for pt in trajectory_points):
            last_point = self._get_current_position(joint_names)
            move_time = 0.0
            for point in trajectory_points:
                diffs = map(operator.sub, point.positions,
                            last_point)
                diffs = map(operator.abs, diffs)
                dflt_vel= [self._dflt_vel[jnt] for jnt in joint_names]
                move_time = move_time + max(map(operator.div, diffs, dflt_vel))
                point.time_from_start = rospy.Duration(move_time)
                last_point = point.positions

        def interp(a, b, pct):
            return a + (b - a) * pct

        def interp_positions(p1, p2, pct):
            return map(interp, p1.positions, p2.positions, [pct] *
                       len(p1.positions))

        end_time = trajectory_points[-1].time_from_start.to_sec()
        control_rate = rospy.Rate(self._control_rate)

        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]

         # Wait for the specified execution time, if not provided use now
        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()
        dataflow.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float('inf')
        )

        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        now_from_start = rospy.get_time() - start_time
        while now_from_start < end_time + (1.0 / self._control_rate):
            idx = bisect.bisect(pnt_times, now_from_start)

            if idx == 0:
                # If our current time is before the first specified point
                # in the trajectory, then we should interpolate between
                # our current position and that point.
                p1 = JointTrajectoryPoint()
                p1.positions = self._get_current_position(joint_names)
            else:
                p1 = trajectory_points[idx - 1]

            if idx != num_points:
                p2 = trajectory_points[idx]
                pct = ((now_from_start - p1.time_from_start.to_sec()) /
                       (p2.time_from_start - p1.time_from_start).to_sec())
                point = interp_positions(p1, p2, pct)
            else:
                # If the current time is after the last trajectory point,
                # just hold that position.
                point = p1.positions


            if not self._command_velocities(joint_names, point):
                return

            control_rate.sleep()
            now_from_start = rospy.get_time() - start_time

        # Keep trying to meet goal until goal_time constraint expired
        last_point = trajectory_points[-1].positions
        last_time = trajectory_points[-1].time_from_start.to_sec()
        def check_goal_state():
            for error in self._get_current_error(joint_names, last_point):
                if (self._goal_error[error[0]] > 0
                    and self._goal_error[error[0]] < math.fabs(error[1])):
                    return error[0]
            else:
                return None

        while ((rospy.get_time() < start_time + last_time + self._goal_time)
               and check_goal_state()):
            if not self._command_velocities(joint_names, last_point):
                return
            control_rate.sleep()

        # Verify goal constraint
        result = check_goal_state()
        if result != None:
            self._command_stop(goal.trajectory.joint_names)
            rospy.logerr("%s: Exceeded Goal Threshold Error %s" %
                         (self._action_name, result,))
            self._server.set_aborted()
        else:
            self._command_stop(goal.trajectory.joint_names)
            self._server.set_succeeded()
