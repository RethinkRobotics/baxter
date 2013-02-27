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
        self._joint_velocity = {}
        self._joint_effort = {}
        self._joint_gc_effort = {}

        ns = '/robot/limb/' + limb + '/'

        self._mode_pub = rospy.Publisher(
            ns + 'joint_command_mode',
            baxter_msgs.msg.JointCommandMode)

        self._command_pub = rospy.Publisher(
            ns + 'command_joint_angles',
            baxter_msgs.msg.JointPositions)

        self._velocity_pub = rospy.Publisher(
            ns + 'command_joint_velocities',
            baxter_msgs.msg.JointVelocities)

        self._joint_states_sub = rospy.Subscriber(
            ns + 'joint_states',
            sensor_msgs.msg.JointState,
            self._joint_states_callback)

#        self._gc_torques_sub = rospy.Subscriber(
#            ns + 'gc_torques',
#            sensor_msgs.msg.JointState,
#            self._gc_torques_callback)

        self._last_state_time = None
        self._state_rate = 0

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if len(self._joint_angle.keys()): #and len(self._joint_gc_effort.keys()):
                break
            rate.sleep()

    def _joint_states_callback(self, msg):
        now = rospy.Time.now()
        if self._last_state_time:
            #lowpassed, but still seems all over the place
            self._state_rate = ((9 * self._state_rate) + (1.0 / (now - self._last_state_time).to_sec()))/10.0
        self._last_state_time = now
        for i in range(len(msg.name)):
            self._joint_angle[msg.name[i]] = msg.position[i]
            self._joint_velocity[msg.name[i]] = msg.velocity[i]
            self._joint_effort[msg.name[i]] = msg.effort[i]

    def _gc_torques_callback(self, msg):
        for i in range(len(msg.name)):
            self._joint_gc_effort[msg.name[i]] = msg.effort[i]

    def set_position_mode(self):
        msg = baxter_msgs.msg.JointCommandMode()
        msg.mode = baxter_msgs.msg.JointCommandMode.POSITION
        self._mode_pub.publish(msg)

    def set_velocity_mode(self):
        msg = baxter_msgs.msg.JointCommandMode()
        msg.mode = baxter_msgs.msg.JointCommandMode.VELOCITY
        self._mode_pub.publish(msg)

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

        Commands the joints of this limb to the specifies velocities
        DANGER: you can break the robot with this.
        """
        msg = baxter_msgs.msg.JointVelocities()
        msg.names = velocities.keys()
        msg.velocities = velocities.values()
        self.set_velocity_mode()
        self._velocity_pub.publish(msg)

    def set_pose(self, pose):
        """
        @param pose dict({str:float})   - dictionary of joint_name:angle

        Commands the limb to the provided pose.  Waits until the reported
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
                if not joint in self._joint_angle:
                  joint = "%s_%s" % (self.name, joint,)
                if joint in self._joint_angle:
                    if abs(angle - self._joint_angle[joint]) >= JOINT_ANGLE_TOLERANCE:
                        good_enough = False
                        rate.sleep()
                        continue
                else:
                    print("unknown joint %s" %  (joint,))


