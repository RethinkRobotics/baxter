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
Baxter RSDK Gripper Action Server
"""
import argparse

import rospy

from dynamic_reconfigure.server import Server

from baxter_interface.cfg import (
    GripperActionServerConfig
)
from gripper_action.gripper_action import (
    GripperActionServer,
)


def start_server(gripper):
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_gripper_action_server%s" %
                    ("" if gripper == 'both' else "_" + gripper,))
    print("Initializing gripper action server...")

    dynamic_cfg_srv = Server(GripperActionServerConfig,
                             lambda config, level: config)

    if gripper == 'both':
        GripperActionServer('right', dynamic_cfg_srv)
        GripperActionServer('left', dynamic_cfg_srv)
    else:
        GripperActionServer(gripper, dynamic_cfg_srv)
    print("Running. Ctrl-c to quit")
    rospy.spin()


def main():
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument("-g", "--gripper", dest="gripper", default="both",
                        choices=['both', 'left', 'right'],
                        help="gripper action server limb",)
    args = parser.parse_args(rospy.myargv()[1:])
    start_server(args.gripper)


if __name__ == "__main__":
    main()
