#!/usr/bin/env python

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
Baxter RSDK Gripper Action Client Example
"""
import sys
import argparse

import rospy

import actionlib

from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)

import baxter_interface


class GripperClient(object):
    def __init__(self, gripper):
        ns = 'robot/end_effector/' + gripper + '_gripper/'
        self._client = actionlib.SimpleActionClient(
            ns + "gripper_action",
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - %s Gripper Action Server Not Found" %
                         (gripper.capitalize(),))
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self):
        self._client.wait_for_result()
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-g', '--gripper', dest='gripper', required=True,
        choices=['left', 'right'],
        help='which gripper to send action commands'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    gripper = args.gripper

    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_gripper_action_test_%s" % (gripper,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    gc = GripperClient(gripper)
    gc.command(position=0.0, effort=50.0)
    gc.wait()
    gc.command(position=100.0, effort=50.0)
    gc.wait()
    gc.command(position=25.0, effort=40.0)
    gc.wait()
    gc.command(position=75.0, effort=20.0)
    gc.wait()
    gc.command(position=0.0, effort=30.0)
    gc.wait()
    gc.command(position=100.0, effort=40.0)
    print gc.wait()
    print "Exiting - Gripper Action Test Example Complete"

if __name__ == "__main__":
    main()
