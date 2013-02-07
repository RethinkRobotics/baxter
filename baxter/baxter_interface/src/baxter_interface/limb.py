import sys
import time

import roslib
roslib.load_manifest('baxter_interface')
import rospy

import baxter_msgs.msg
import sensor_msgs.msg

JOINT_ANGLE_TOLERANCE = 0.0872664626

class Limb(object):
    def __init__(self, limb):
        """
        Interface class for a limb on the Baxter robot.

        @param limb - limb to interface
        """
        self.name = limb
        self._joint_angle = {}
        self._joint_effort = {}
        self._joint_gc_effort = {}

        ns = '/robot/limb/' + limb + '/'

        self._mode_pub = rospy.Publisher(
            ns + 'joint_command_mode',
            baxter_msgs.msg.JointCommandMode)

        self._command_pub = rospy.Publisher(
            ns + 'command_joint_angles',
            baxter_msgs.msg.JointPositions)

        self._joint_states_sub = rospy.Subscriber(
            ns + 'joint_states',
            sensor_msgs.msg.JointState,
            self._joint_states_callback)

        self._gc_torques_sub = rospy.Subscriber(
            ns + 'gc_torques',
            sensor_msgs.msg.JointState,
            self._gc_torques_callback)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if len(self._joint_angle.keys()) and len(self._joint_gc_effort.keys()):
                break
            rate.sleep()

    def _joint_states_callback(self, msg):
        for i in range(len(msg.name)):
            self._joint_angle['%s_%s' % (self.name, msg.name[i])] = msg.position[i]
            self._joint_effort['%s_%s' % (self.name, msg.name[i])] = msg.effort[i]

    def _gc_torques_callback(self, msg):
        for i in range(len(msg.name)):
            self._joint_gc_effort[msg.name[i]] = msg.effort[i]

    def set_position_mode(self):
        msg = baxter_msgs.msg.JointCommandMode()
        msg.mode = baxter_msgs.msg.JointCommandMode.POSITION
        self._mode_pub.publish(msg)

    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @param joint    - name of a joint
        """
        return self._joint_angle[joint]

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

    def set_pose(self, pose):
        """
        @param pose dict({str:float})   - dictionary of joint_name:angle

        Commands the robot to the provided pose.  Waits until the reported
        joint state matches that specified.
        """
        msg = baxter_msgs.msg.JointPositions()
        for joint, angle in pose.items():
            msg.names.append(joint)
            msg.angles.append(angle)

        good_enough = False
        rate = rospy.Rate(100)
        while not good_enough and not rospy.is_shutdown():
            self.set_position_mode()
            self._command_pub.publish(msg)

            good_enough = True
            for joint, angle in pose.items():
                if abs(angle - self._joint_angle[joint]) >= JOINT_ANGLE_TOLERANCE:
                    good_enough = False
                    rate.sleep()
                    continue


