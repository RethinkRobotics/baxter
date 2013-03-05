import sys
import time
import copy

import roslib
roslib.load_manifest('baxter_interface')
import rospy

import baxter_msgs.msg as baxmsg
import std_msgs.msg as stdmsg

class Gripper(object):
    def __init__(self, limb):
        """
        Interface class for a hand on the Baxter robot.

        @param limb - limb to interface
        """
        self.name = limb

        ns = '/robot/limb/' + limb + '/accessory/gripper/'
        sdkns = '/sdk' + ns

        self._pub_enable = rospy.Publisher(ns + 'set_enabled', stdmsg.Bool)
        self._pub_reset = rospy.Publisher(ns + 'command_reset', stdmsg.Bool)
        self._pub_calibrate = rospy.Publisher(ns + 'command_calibrate', stdmsg.Empty)
        self._pub_command = rospy.Publisher(sdkns + 'command_set', baxmsg.GripperCommand)

        self._sub_identity = rospy.Subscriber(sdkns + 'identity',
                                             baxmsg.GripperIdentity,
                                             self.on_gripper_identity,
                                            )
        self._sub_properties = rospy.Subscriber(sdkns + 'properties',
                                               baxmsg.GripperProperties,
                                               self.on_gripper_properties,
                                              )
        self._sub_state = rospy.Subscriber(sdkns + 'state',
                                          baxmsg.GripperState,
                                          self.on_gripper_state,
                                         )
        self._command = baxmsg.GripperCommand(position=0.0,
                                       force=30.0,
                                       velocity=100.0,
                                       holding=0.0,
                                       deadZone=3.0)
        self._identity = baxmsg.GripperIdentity()
        self._properties = baxmsg.GripperProperties()
        self._state = baxmsg.GripperState()

    def _clip(self, val):
        return max(min(val, 100.0), 0.0)

    def enable(self):
        self._pub_enable.publish(True)

    def disable(self):
        self._pub_enable.publish(False)

    def reset(self):
        self._pub_reset.publish(False)

    def reboot(self):
        self._pub_reset.publish(True)

    def calibrate(self):
        self._pub_calibrate.publish(stdmsg.Empty())

    def stop(self):
        cmd = baxmsg.GripperCommand()
        cmd.position = self._state.position
        cmd.velocity = 0.0
        cmd.force = self._state.force
        cmd.holding = self._state.force
        cmd.deadZone = self._command.deadZone
        self._pub_command.publish(cmd)

    def set_position(self, position):
        self._command.position = self._clip(position)
        self._pub_command.publish(self._command)

    def set_velocity(self, velocity):
        self._command.velocity = self._clip(velocity)
        self._pub_command.publish(self._command)

    def set_force(self, force):
        self._command.force = self._clip(force)
        self._pub_command.publish(self._command)

    def set_holding_force(self, force):
        self._command.holding = self._clip(force)
        self._pub_command.publish(self._command)

    def set_dead_band(self, dead_band):
        self._command.deadZone = self._clip(dead_band)
        self._pub_command.publish(self._command)

    def inc_position(self, position):
        self._command.position = self._clip(position + self._command.position)
        self._pub_command.publish(self._command)

    def inc_velocity(self, velocity):
        self._command.velocity = self._clip(velocity + self._command.velocity)
        self._pub_command.publish(self._command)

    def inc_force(self, force):
        self._command.force = self._clip(force + self._command.force)
        self._pub_command.publish(self._command)

    def inc_holding_force(self, force):
        self._command.holding = self._clip(force + self._command.holding)
        self._pub_command.publish(self._command)

    def inc_dead_band(self, dead_band):
        self._command.deadZone = self._clip(dead_band + self._command.deadZone)
        self._pub_command.publish(self._command)

    def open(self):
        self.set_position(100.0)

    def close(self):
        self.set_position(0.0)

    def on_gripper_state(self, state):
        self._state = copy.deepcopy(state)

    def on_gripper_identity(self, identity):
        self._identity = copy.deepcopy(identity)

    def on_gripper_properties(self, properties):
        self._properties = copy.deepcopy(properties)

