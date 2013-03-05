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

        ns = '/robot/limb/' + limb + '/'

        self._pub_enable = rospy.Publisher(ns + 'set_enabled', stdmsg.Bool)
        self._pub_reset = rospy.Publisher(ns + 'command_reset', stdmsg.Bool)
        self._pub_calibrate = rospy.Publisher(ns + 'command_calibrate', stdmsg.Empty)
        self._pub_command = rospy.Publisher(ns + 'command_set', baxmsg.GripperCommand)

        self._sub_identity = rospy.Subscriber(ns + 'identity',
                                             baxmsg.GripperIdentity,
                                             self.on_gripper_identity,
                                            )
        self._sub_properties = rospy.Subscriber(ns + 'properties',
                                               baxmsg.GripperProperties,
                                               self.on_gripper_properties,
                                              )
        self._sub_state = rospy.Subscriber(ns + 'state',
                                          baxmsg.GripperState,
                                          self.on_gripper_state,
                                         )
        self._command = GripperCommand(position=0.0,
                                       force=30.0,
                                       velocity=100.0,
                                       holding=0.0,
                                       deadZone=3.0)
        self._identity = GripperIdentity()
        self._properties = GripperProperties()
        self._state = GripperState()

    def enable(self):
        self._pub_enable.publish(True)

    def disable(self):
        self._pub_enable.publish(False)

    def reset(self):
        self._pub_reset.publish(False)

    def reboot(self):
        self._pub_reset.publish(True)

    def calibrate(self):
        self.pub_calibrate.publish(stdmsg.Empty())

    def stop(self):
        cmd = baxmsg.GripperCommand()
        cmd.position = self._position
        cmd.velocity = 0.0
        cmd.force = self._state.force
        cmd.holding = self._state.force
        cmd.deadZone = self._command.dead_band
        self.pub_command.publish(cmd)

    def set_velocity(self, velocity):
        self._command.velocity = velocity
        self.pub_command.publish(cmd)

    def set_force(self, force):
        self._command.force = force
        self.pub_command.publish(cmd)

    def set_holding_force(self, force):
        self._command.holding = force
        self.pub_command.publish(cmd)

    def set_dead_band(self, dead_band):
        self._command.deadZone = dead_band
        self.pub_command.publish(cmd)

    def on_gripper_state(self, state):
        self._state = copy.deepcopy(state)

    def on_gripper_identity(self, identity):
        self._identity = copy.deepcopy(identity)

    def on_gripper_properties(self, properties):
        self._properties = copy.deepcopy(properties)

