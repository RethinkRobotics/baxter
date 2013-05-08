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

import roslib
roslib.load_manifest('baxter_interface')
import rospy

import baxter_msgs.msg
import sensor_msgs.msg

import settings
import dataflow

class Limb(object):
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

        ns = '/robot/limb/' + limb + '/'
        sdkns = '/sdk' + ns

        self._pub_joint_mode = rospy.Publisher(
            ns + 'joint_command_mode',
            baxter_msgs.msg.JointCommandMode)

        self._pub_joint_position = rospy.Publisher(
            ns + 'command_joint_angles',
            baxter_msgs.msg.JointPositions)

        self._pub_joint_velocity = rospy.Publisher(
            ns + 'command_joint_velocities',
            baxter_msgs.msg.JointVelocities)

        joint_state_sub = rospy.Subscriber(
            ns + 'joint_states',
            sensor_msgs.msg.JointState,
            self._on_joint_states)

        cartesian_state_sub = rospy.Subscriber(
            sdkns + 'endpoint/state',
            baxter_msgs.msg.EndpointState,
            self._on_endpoint_states)

        self._last_state_time = None
        self._state_rate = 0

        dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0)

    def _on_joint_states(self, msg):
        now = rospy.Time.now()
        if self._last_state_time:
            #cheap low pass
            rate = (1.0 / (now - self._last_state_time).to_sec())
            self._state_rate = ((99 * self._state_rate) + rate)/100
        self._last_state_time = now
        for i in range(len(msg.name)):
            self._joint_angle[msg.name[i]] = msg.position[i]
            self._joint_velocity[msg.name[i]] = msg.velocity[i]
            self._joint_effort[msg.name[i]] = msg.effort[i]

    def _on_endpoint_states(self, msg):
        self._cartesian_pose = msg.Pose
        self._cartesian_velocity = msg.Twist
        self._cartesian_effort = msg.Wrench

    def joints(self):
        """
        Return the names of the joints for which data has been received
        """
        return self._joint_angle.keys()

    def state_rate(self):
        """
        Return the rate at which joint state has been received
        """
        return self._state_rate

    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @param joint    - name of a joint
        """
        return self._joint_angle[joint]

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        @param joint    - name of a joint
        """
        return self._joint_velocity[joint]

    def joint_effort(self, joint):
        """
        Return the requested joint effort.

        @param joint    - name of a joint
        """
        return self._joint_effort[joint]

    def set_joint_position_mode(self):
        """
        Set the joint controller in position mode
        """
        msg = baxter_msgs.msg.JointCommandMode()
        msg.mode = baxter_msgs.msg.JointCommandMode.POSITION
        self._pub_joint_mode.publish(msg)

    def set_joint_velocity_mode(self):
        """
        Set the joint controller in velocity mode
        """
        msg = baxter_msgs.msg.JointCommandMode()
        msg.mode = baxter_msgs.msg.JointCommandMode.VELOCITY
        self._pub_joint_mode.publish(msg)

    def set_joint_positions(self, positions):
        """
        @param positions dict({str:float})   - dictionary of joint_name:angle

        Commands the joints of this limb to the specified positions
        """
        msg = baxter_msgs.msg.JointPositions()
        msg.names = positions.keys()
        msg.angles = positions.values()
        self.set_joint_position_mode()
        self._pub_joint_position.publish(msg)

    def set_joint_velocities(self, velocities):
        """
        @param velocities dict({str:float})   - dictionary of joint_name:velocity

        Commands the joints of this limb to the specified velocities
        """
        msg = baxter_msgs.msg.JointVelocities()
        msg.names = velocities.keys()
        msg.velocities = velocities.values()
        self.set_joint_velocity_mode()
        self._pub_joint_velocity.publish(msg)

    def set_neutral_pose(self):
        """
        Command the joints to the center of their joint ranges
        """
        angles = dict(zip(
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2'],
            [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))

        return self.move_to_joint_positions(angles)

    def move_to_joint_positions(self, pose, timeout=15.0):
        """
        @param pose dict({str:float})   - dictionary of joint_name:angle
        @param timeout    - seconds to wait for move to finish [10]

        Commands the limb to the provided pose.  Waits until the reported
        joint state matches that specified.
        """
        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._joint_angle[joint])
            return joint_diff

        diffs = [genf(j,a) for j,a in pose.items() if j in self._joint_angle]

        dataflow.wait_for(
            lambda: not any(diff() >= settings.JOINT_ANGLE_TOLERANCE for diff in diffs),
            timeout=timeout,
            rate=100,
            body=lambda: self.set_joint_positions(pose)
            )