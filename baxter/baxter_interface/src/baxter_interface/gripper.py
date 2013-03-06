import sys
import time
import copy

import roslib
roslib.load_manifest('baxter_interface')
import rospy

import baxter_msgs.msg as baxmsg
import std_msgs.msg as stdmsg

class Gripper(object):
    def __init__(self, gripper):
        """
        Interface class for a gripper on the Baxter robot.

        @param gripper - gripper to interface
        """
        self.name = gripper

        ns = '/robot/limb/' + self.name + '/accessory/gripper/'
        sdkns = '/sdk' + ns

        self._pub_enable = rospy.Publisher(
            ns + 'set_enabled',
            stdmsg.Bool)

        self._pub_reset = rospy.Publisher(
            ns + 'command_reset',
            stdmsg.Bool)

        self._pub_calibrate = rospy.Publisher(
            ns + 'command_calibrate',
            stdmsg.Empty)

        self._pub_command = rospy.Publisher(
            sdkns + 'command_set',
            baxmsg.GripperCommand)

        self._sub_identity = rospy.Subscriber(
            sdkns + 'identity',
            baxmsg.GripperIdentity,
            self._on_gripper_identity)

        self._sub_properties = rospy.Subscriber(
            sdkns + 'properties',
            baxmsg.GripperProperties,
            self._on_gripper_properties)

        self._sub_state = rospy.Subscriber(
            sdkns + 'state',
            baxmsg.GripperState,
            self._on_gripper_state)

        self._command = baxmsg.GripperCommand(
            position=0.0,
            force=30.0,
            velocity=100.0,
            holding=0.0,
            deadZone=3.0)

        self._identity = baxmsg.GripperIdentity()
        self._properties = baxmsg.GripperProperties()
        self._state = baxmsg.GripperState()

    def _on_gripper_state(self, state):
        self._state = copy.deepcopy(state)

    def _on_gripper_identity(self, identity):
        self._identity = copy.deepcopy(identity)

    def _on_gripper_properties(self, properties):
        self._properties = copy.deepcopy(properties)

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)


    def enable(self):
        """
        Enable the gripper
        """
        self._pub_enable.publish(True)

    def disable(self):
        """
        Disable the gripper
        """
        self._pub_enable.publish(False)

    def reset(self):
        """
        Reset the gripper
        """
        self._pub_reset.publish(False)

    def reboot(self):
        """
        Reboot the gripper
        """
        self._pub_reset.publish(True)

    def calibrate(self):
        """
        Calibrate the gripper
        """
        self._pub_calibrate.publish(stdmsg.Empty())

    def stop(self):
        """
        Stop the gripper at the current position and force
        """
        cmd = baxmsg.GripperCommand()
        cmd.position = self._state.position
        cmd.velocity = 0.0
        cmd.force = self._state.force
        cmd.holding = self._state.force
        cmd.deadZone = self._command.deadZone
        self._pub_command.publish(cmd)

    def set_position(self, position):
        """
        Set the gripper position

        @param position (float) - in % 0=close 100=open
        """
        self._command.position = self._clip(position)
        self._pub_command.publish(self._command)

    def set_velocity(self, velocity):
        """
        Set the gripper velocity

        @param velocity (float) - in % 0=stop 100=max
        """
        self._command.velocity = self._clip(velocity)
        self._pub_command.publish(self._command)

    def set_force(self, force):
        """
        Set the gripper force

        @param force (float) - in % 0=none 100=max
        """
        self._command.force = self._clip(force)
        self._pub_command.publish(self._command)

    def set_holding_force(self, force):
        """
        Set the gripper holding force

        @param force (float) - in % 0=none 100=max
        """
        self._command.holding = self._clip(force)
        self._pub_command.publish(self._command)

    def set_dead_band(self, dead_band):
        """
        Set the gripper dead band

        @param dead_band (float) - in % of full position
        """
        self._command.deadZone = self._clip(dead_band)
        self._pub_command.publish(self._command)

    def inc_position(self, position):
        """
        Increment the gripper position

        @param position (float) - percentage to increment by
        """
        self._command.position = self._clip(position + self._command.position)
        self._pub_command.publish(self._command)

    def inc_velocity(self, velocity):
        """
        Increment the gripper velocity

        @param velocity (float) - percentage to increment by
        """
        self._command.velocity = self._clip(velocity + self._command.velocity)
        self._pub_command.publish(self._command)

    def inc_force(self, force):
        """
        Increment the gripper force

        @param force (float) - percentage to increment by
        """
        self._command.force = self._clip(force + self._command.force)
        self._pub_command.publish(self._command)

    def inc_holding_force(self, force):
        """
        Increment the gripper holding force

        @param force (float) - percentage to increment by
        """
        self._command.holding = self._clip(force + self._command.holding)
        self._pub_command.publish(self._command)

    def inc_dead_band(self, dead_band):
        """
        Increment the gripper dead band

        @param dead_band (float) - percentage to increment by
        """
        self._command.deadZone = self._clip(dead_band + self._command.deadZone)
        self._pub_command.publish(self._command)

    def open(self, timeout=0):
        self.set_position(100.0)

    def close(self, timeout=0):
        self.set_position(0.0)


