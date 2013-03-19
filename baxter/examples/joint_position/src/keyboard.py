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
Baxter RSDK Joint Position Example: keyboard
"""
import roslib
roslib.load_manifest('joint_position')
import rospy

import baxter_interface
import iodevices

def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left')
    grip_right = baxter_interface.Gripper('right')
    set_j = lambda l,j,d: l.set_positions({j: d + l.joint_angle(j)})
    bindings = {
    #   key: (function, args, description)
        '6': (set_j, [left, 's0', 0.1], "left s0 increase"),
        '9': (set_j, [left, 's0', -0.1], "left s0 decrease"),
        '7': (set_j, [left, 's1', 0.1], "left s1 increase"),
        '8': (set_j, [left, 's1', -0.1], "left s1 decrease"),
        'y': (set_j, [left, 'e0', 0.1], "left e0 increase"),
        'o': (set_j, [left, 'e0', -0.1], "left e0 decrease"),
        'u': (set_j, [left, 'e1', 0.1], "left e1 increase"),
        'i': (set_j, [left, 'e1', -0.1], "left e1 decrease"),
        'h': (set_j, [left, 'w0', 0.1], "left w0 increase"),
        'l': (set_j, [left, 'w0', -0.1], "left w0 decrease"),
        'j': (set_j, [left, 'w1', 0.1], "left w1 increase"),
        'k': (set_j, [left, 'w1', -0.1], "left w1 decrease"),
        'n': (set_j, [left, 'w2', 0.1], "left w2 increase"),
        '.': (set_j, [left, 'w2', -0.1], "left w2 decrease"),
        'm': (grip_left.close, [], "left: gripper close"),
        ',': (grip_left.open, [], "left: gripper open"),

        '1': (set_j, [right, 's0', 0.1], "right s0 increase"),
        '4': (set_j, [right, 's0', -0.1], "right s0 decrease"),
        '2': (set_j, [right, 's1', 0.1], "right s1 increase"),
        '3': (set_j, [right, 's1', -0.1], "right s1 decrease"),
        'q': (set_j, [right, 'e0', 0.1], "right e0 increase"),
        'r': (set_j, [right, 'e0', -0.1], "right e0 decrease"),
        'w': (set_j, [right, 'e1', 0.1], "right e1 increase"),
        'e': (set_j, [right, 'e1', -0.1], "right e1 decrease"),
        'a': (set_j, [right, 'w0', 0.1], "right w0 increase"),
        'f': (set_j, [right, 'w0', -0.1], "right w0 decrease"),
        's': (set_j, [right, 'w1', 0.1], "right w1 increase"),
        'd': (set_j, [right, 'w1', -0.1], "right w1 decrease"),
        'z': (set_j, [right, 'w2', 0.1], "right w2 increase"),
        'v': (set_j, [right, 'w2', -0.1], "right w2 decrease"),
        'x': (grip_right.close, [], "right: gripper close"),
        'c': (grip_right.open, [], "right: gripper open"),
     }
    done = False
    while not done and not rospy.is_shutdown():
        c = iodevices.getch()
        if c:
            if c in ['\x1b', '\x03']:
                done = True
            elif c in bindings:
                cmd = bindings[c]
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(), key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))


def main():
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    map_keyboard()

    print("Disabling robot... ")
    rs.disable()

if __name__ == '__main__':
    main()
