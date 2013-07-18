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
Baxter RSDK Gripper Example: keyboard
"""
import roslib
roslib.load_manifest('gripper_control')
import rospy

import baxter_interface
import iodevices

def map_keyboard():
    left = baxter_interface.Gripper('left')
    right = baxter_interface.Gripper('right')
    bindings = {
    #   key: (function, args, description)
        'r': (left.reboot, [], "left: reset"),
        'R': (right.reboot, [], "right: reset"),
        'c': (left.calibrate, [], "left: calibrate"),
        'C': (right.calibrate, [], "right: calibrate"),
        'q': (left.close, [], "left: close"),
        'Q': (right.close, [], "right: close"),
        'w': (left.open, [], "left: open"),
        'W': (right.open, [], "right: open"),
        '[': (left.set_velocity, [100.0], "left: set 100% velocity"),
        '{': (right.set_velocity, [100.0], "right: set 100% velocity"),
        ']': (left.set_velocity, [30.0], "left: set 30% velocity"),
        '}': (right.set_velocity, [30.0], "right: set 30% velocity"),
        's': (left.stop, [], "left: stop"),
        'S': (right.stop, [], "right: stop"),
        'z': (left.inc_dead_band, [-1.0], "left: decrease dead band"),
        'Z': (right.inc_dead_band, [-1.0], "right: decrease dead band"),
        'x': (left.inc_dead_band, [1.0], "left: increase dead band"),
        'X': (right.inc_dead_band, [1.0], "right: increase dead band"),
        'f': (left.inc_force, [-5.0], "left: decrease moving force"),
        'F': (right.inc_force, [-5.0], "right:  decrease moving force"),
        'g': (left.inc_force, [5.0], "left:  increase moving force"),
        'G': (right.inc_force, [5.0], "right:  increase moving force"),
        'h': (left.inc_holding_force, [-5.0], "left:  decrease holding force"),
        'H': (right.inc_holding_force, [-5.0], "right:  decrease holding force"),
        'j': (left.inc_holding_force, [5.0], "left:  increase holding force"),
        'J': (right.inc_holding_force, [5.0], "right:  increase holding force"),
        'v': (left.inc_velocity, [-5.0], "left:  decrease velocity"),
        'V': (right.inc_velocity, [-5.0], "right:  decrease velocity"),
        'b': (left.inc_velocity, [5.0], "left:  increase velocity"),
        'B': (right.inc_velocity, [5.0], "right:  increase velocity"),
        'u': (left.inc_position, [-5.0], "left:  decrease position"),
        'U': (right.inc_position, [-5.0], "right:  decrease position"),
        'i': (left.inc_position, [5.0], "left:  increase position"),
        'I': (right.inc_position, [5.0], "right:  increase position"),
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
    rospy.init_node("rethink_rsdk_gripper_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    map_keyboard()

    print("Disabling robot... ")
    rs.disable()

if __name__ == '__main__':
    main()
