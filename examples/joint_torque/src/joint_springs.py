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
Baxter RSDK Joint Torque Example: joint springs
Moves the specified limb to a neutral location and enters torque control mode
Attaching virtual springs (Hooke's law) to each joint maintaining the start
position
"""

import argparse

import roslib
roslib.load_manifest('joint_torque')
import rospy

from dynamic_reconfigure.server import Server
from joint_torque.cfg import (
    JointSpringsExampleConfig
)
from std_msgs.msg import (
    Empty,
)

import baxter_interface

class JointSprings(object):
    """
    @param limb - limb on which to run joint springs example
    @param reconfig_server - dynamic reconfigure server

    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode,
    and attaching virtual springs.
    """
    def __init__(self, limb, reconfig_server):
        self._dyn = reconfig_server
        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # initialize parameters
        self._springs = dict()
        self._start_angles = dict()

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] +
                                                    '_spring_stiffness']

    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server
        """
        # get latest spring constants
        self._update_parameters()

        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles
        cur_pos = self._limb.joint_angles()
        # calculate current forces
        for joint in self._start_angles.keys():
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
                                                   cur_pos[joint])
        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location
        """
        self._limb.move_to_neutral()

    def attach_springs(self):
        """ 
        Switches to joint torque mode and attached joint springs to current
        joint positions
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        rate = 1000 #Hz
        control_rate = rospy.Rate(rate)

        # for safety purposes set command timeout
        # if 20 command cycles missed - timeout and disable robot
        self._limb.set_command_timeout(1.0 / (rate / 20))

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown() and self._rs.state().enabled:
            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
            """
            Switches out of joint torque mode to exit cleanly
            """
            self._limb.exit_control_mode()

def main(limb):
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_torque_springs_%s" % (limb,))
    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config,level: config)
    js = JointSprings(limb, dynamic_cfg_srv)
    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)
    js.move_to_neutral()
    js.attach_springs()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", "--limb", dest="limb", required=True,
                    choices=['left', 'right'],
                    help="limb on which to attach joint springs")
    args = parser.parse_args()
    main(args.limb)
