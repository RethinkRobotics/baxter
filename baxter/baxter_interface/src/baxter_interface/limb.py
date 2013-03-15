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
        self._joint_gc_effort = {}

        ns = '/robot/limb/' + limb + '/'

        self._pub_mode = rospy.Publisher(
            ns + 'joint_command_mode',
            baxter_msgs.msg.JointCommandMode)

        self._pub_position = rospy.Publisher(
            ns + 'command_joint_angles',
            baxter_msgs.msg.JointPositions)

        self._pub_velocity = rospy.Publisher(
            ns + 'command_joint_velocities',
            baxter_msgs.msg.JointVelocities)

        self._sub_joint_states = rospy.Subscriber(
            ns + 'joint_states',
            sensor_msgs.msg.JointState,
            self._callback_joint_states)

        self._last_state_time = None
        self._state_rate = 0

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if len(self._joint_angle.keys()):
                break
            rate.sleep()

    def _callback_joint_states(self, msg):
        now = rospy.Time.now()
        if self._last_state_time:
            self._state_rate = (1.0 / (now - self._last_state_time).to_sec())
        self._last_state_time = now
        for i in range(len(msg.name)):
            self._joint_angle[msg.name[i]] = msg.position[i]
            self._joint_velocity[msg.name[i]] = msg.velocity[i]
            self._joint_effort[msg.name[i]] = msg.effort[i]

    def _callback_gc_torques(self, msg):
        for i in range(len(msg.name)):
            self._joint_gc_effort[msg.name[i]] = msg.effort[i]

    def set_position_mode(self):
        msg = baxter_msgs.msg.JointCommandMode()
        msg.mode = baxter_msgs.msg.JointCommandMode.POSITION
        self._pub_mode.publish(msg)

    def set_velocity_mode(self):
        msg = baxter_msgs.msg.JointCommandMode()
        msg.mode = baxter_msgs.msg.JointCommandMode.VELOCITY
        self._pub_mode.publish(msg)

    def state_rate(self):
        """
        Return the rate at which join state has been received
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

    def joint_gc_effort(self, joint):
        """
        Return the requested joint gravity comp effort.

        @param joint    - name of a joint
        """
        return self._joint_gc_effort[joint]

    def set_velocities(self, velocities):
        """
        @param velocities dict({str:float})   - dictionary of joint_name:velocity

        Commands the joints of this limb to the specified velocities
        """
        msg = baxter_msgs.msg.JointVelocities()
        msg.names = velocities.keys()
        msg.velocities = velocities.values()
        self.set_velocity_mode()
        self._pub_velocity.publish(msg)

    def set_positions(self, positions):
        """
        @param positions dict({str:float})   - dictionary of joint_name:angle

        Commands the joints of this limb to the specified positions
        """
        msg = baxter_msgs.msg.JointPositions()
        msg.names = positions.keys()
        msg.angles = positions.values()
        self.set_position_mode()
        self._pub_position.publish(msg)

    def set_pose(self, pose):
        """
        @param pose dict({str:float})   - dictionary of joint_name:angle

        Commands the limb to the provided pose.  Waits until the reported
        joint state matches that specified.
        """
        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._joint_angle[joint])
            return joint_diff

        diffs = [genf(j,a) for j,a in pose.items() if j in self._joint_angle]

        rate = rospy.Rate(100)
        while any(diff() >= settings.JOINT_ANGLE_TOLERANCE for diff in diffs):
            self.set_positions(pose)
            rate.sleep()
            if rospy.is_shutdown():
                return
