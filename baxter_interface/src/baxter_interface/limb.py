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

from copy import deepcopy
import collections

import roslib
roslib.load_manifest('baxter_interface')
import rospy

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)

from sensor_msgs.msg import (
    JointState
)

from std_msgs.msg import (
    Float64,
)

import settings
import dataflow

class Limb(object):
    # Containers
    Point = collections.namedtuple('Point', ['x', 'y', 'z'])
    Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

    def __init__(self, limb):
        """
        Interface class for a limb on the Baxter robot.

        @param limb - limb to interface
        """
        self.name = limb
        self._joint_angle = {}
        self._joint_velocity = {}
        self._joint_effort = {}
        self._cartesian_pose = {}
        self._cartesian_velocity = {}
        self._cartesian_effort = {}

        self._joint_names = {
            'left': ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                'left_w0', 'left_w1', 'left_w2'],
            'right': ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                'right_w0', 'right_w1', 'right_w2']
            }

        ns = '/robot/limb/' + limb + '/'

        self._command_msg = JointCommand()

        self._pub_speed_ratio = rospy.Publisher(
            ns + 'set_speed_ratio',
            Float64)

        self._pub_joint_cmd = rospy.Publisher(
            ns + 'joint_command',
            JointCommand)

        self._pub_joint_cmd_timeout = rospy.Publisher(
            ns + 'joint_command_timeout',
            Float64)

        self._last_state_time = None
        self._state_rate = 0

        joint_state_sub = rospy.Subscriber(
            '/robot/joint_states',
            JointState,
            self._on_joint_states)

        cartesian_state_sub = rospy.Subscriber(
            ns + 'endpoint/state',
            EndpointState,
            self._on_endpoint_states)

        dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0)

    def _on_joint_states(self, msg):
        now = rospy.Time.now()
        if self._last_state_time:
            # cheap low pass
            rate = (1.0 / (now - self._last_state_time).to_sec())
            self._state_rate = ((99 * self._state_rate) + rate) / 100
        self._last_state_time = now
        for i in range(len(msg.name)):
            if self.name in msg.name[i]:
                self._joint_angle[msg.name[i]] = msg.position[i]
                self._joint_velocity[msg.name[i]] = msg.velocity[i]
                self._joint_effort[msg.name[i]] = msg.effort[i]

    def _on_endpoint_states(self, msg):
        # Comments in this private method are for documentation purposes.
        # _pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
        self._cartesian_pose = {
            'position': self.Point(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
            'orientation': self.Quaternion(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
        }
        # _twist = {'linear': (x, y, z), 'angular': (x, y, z)}
        self._cartesian_velocity = {
            'linear': self.Point(
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            ),
            'angular': self.Point(
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z,
            ),
        }
        # _wrench = {'force': (x, y, z), 'torque': (x, y, z)}
        self._cartesian_effort = {
            'force': self.Point(
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
            ),
            'torque': self.Point(
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ),
        }

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.
        """
        return self._joint_names[self.name]

    def state_rate(self):
        """
        Return the rate at which the joint and Cartesian states are received.
        """
        return self._state_rate

    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @param joint    - name of a joint
        """
        return self._joint_angle[joint]

    def joint_angles(self):
        """
        Return all joint angles.
        """
        return deepcopy(self._joint_angle)

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        @param joint    - name of a joint
        """
        return self._joint_velocity[joint]

    def joint_velocities(self):
        """
        Return all joint velocities.
        """
        return deepcopy(self._joint_velocity)

    def joint_effort(self, joint):
        """
        Return the requested joint effort.

        @param joint    - name of a joint
        """
        return self._joint_effort[joint]

    def joint_efforts(self):
        """
        Return all joint efforts.
        """
        return deepcopy(self._joint_effort)

    def endpoint_pose(self):
        """
        Return Cartesian endpoint pose {position, orientation}.
        """
        return deepcopy(self._cartesian_pose)

    def endpoint_velocity(self):
        """
        Return Cartesian endpoint twist {linear, angular}.
        """
        return deepcopy(self._cartesian_velocity)

    def endpoint_effort(self):
        """
        Return Cartesian endpoint wrench {force, torque}.
        """
        return deepcopy(self._cartesian_effort)

    def set_command_timeout(self, timeout):
        """
        @param timeout float  - float timeout in seconds

        Set the timeout in seconds for the joint controller
        """
        self._pub_joint_cmd_timeout.publish(Float64(timeout))

    def set_joint_position_speed(self, speed):
        """
        @param speed float  - speed ratio of maximum joint speed for execution

        Sets the ratio [0.0-1.0] (clipped) of maximum joint speed for joint
        position control execution. This will be persistent until a new
        execution speed is specified.
        """
        self._pub_speed_ratio.publish(Float64(speed))

    def set_joint_positions(self, positions):
        """
        @param positions dict({str:float})  - dictionary of joint_name:angle

        Commands the joints of this limb to the specified positions.
        """
        self._command_msg.names = positions.keys()
        self._command_msg.command = positions.values()
        self._command_msg.mode = JointCommand.POSITION_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_velocities(self, velocities):
        """
        @param velocities dict({str:float})  - dictionary of joint_name:velocity

        Commands the joints of this limb to the specified velocities.
        Important: set_joint_velocities must be commanded at a rate great than
        the timeout specified by set_command_timeout. If the timeout is
        exceeded before a new set_joint_velocities command is received, the
        controller will switch modes back to position mode for safety purposes.
        """
        self._command_msg.names = velocities.keys()
        self._command_msg.command = velocities.values()
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_torques(self, torques):
        """
        @param torques dict({str:float})  - dictionary of joint_name:torque

        Commands the joints of this limb to the specified torques.
        Important: set_joint_torques must be commanded at a rate great than
        the timeout specified by set_command_timeout. If the timeout is
        exceeded before a new set_joint_torques command is received, the
        controller will switch modes back to position mode for safety purposes.
        """
        self._command_msg.names = torques.keys()
        self._command_msg.command = torques.values()
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def move_to_neutral(self):
        """
        Command the joints to the center of their joint ranges
        """
        angles = dict(zip(self.joint_names(),
                          [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        return self.move_to_joint_positions(angles)

    def move_to_joint_positions(self, positions, timeout=15.0):
        """
        @param positions dict({str:float})  - dictionary of joint_name:angle
        @param timeout    - seconds to wait for move to finish [15]

        Commands the limb to the provided positions.  Waits until the reported
        joint state matches that specified.
        """
        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._joint_angle[joint])
            return joint_diff

        diffs = [genf(j, a) for j, a in positions.items() if
                 j in self._joint_angle]

        dataflow.wait_for(
                          lambda: (not
                                   any(diff() >= settings.JOINT_ANGLE_TOLERANCE
                                       for diff in diffs)),
                          timeout=timeout,
                          rate=100,
                          body=lambda: self.set_joint_positions(positions)
                          )