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
import rospy

import baxter_interface
import baxter_io_devices


def map_keyboard():
    # initialize interfaces
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left')
    right = baxter_interface.Gripper('right')

    def clean_shutdown():
        if not init_state:
            print("Disabling robot...")
            rs.disable()
        print("Exiting example.")
    rospy.on_shutdown(clean_shutdown)

    def l_command(offset):
        left.command_position(left.position() + offset)

    def r_command(offset):
        right.command_position(right.position() + offset)

    def l_holding(offset):
        left.set_holding_force(left.parameters()['holding_force'] + offset)

    def r_holding(offset):
        right.set_holding_force(right.parameters()['holding_force'] + offset)

    def l_moving(offset):
        left.set_moving_force(left.parameters()['moving_force'] + offset)

    def r_moving(offset):
        right.set_moving_force(right.parameters()['moving_force'] + offset)

    def l_velocity(value):
        left.set_velocity(value)

    def r_velocity(value):
        right.set_velocity(value)

    def l_dead_band(offset):
        left.set_dead_band(left.parameters()['dead_band'] + offset)

    def r_dead_band(offset):
        right.set_dead_band(right.parameters()['dead_band'] + offset)

    bindings = {
    #   key: (function, args, description)
        'r': (left.reboot, [], "left: reboot"),
        'R': (right.reboot, [], "right: reboot"),
        'c': (left.calibrate, [], "left: calibrate"),
        'C': (right.calibrate, [], "right: calibrate"),
        'q': (left.close, [], "left: close"),
        'Q': (right.close, [], "right: close"),
        'w': (left.open, [], "left: open"),
        'W': (right.open, [], "right: open"),
        '[': (l_velocity, [100.0], "left: set 100% velocity"),
        '{': (r_velocity, [100.0], "right: set 100% velocity"),
        ']': (l_velocity, [30.0], "left: set 30% velocity"),
        '}': (r_velocity, [30.0], "right: set 30% velocity"),
        's': (left.stop, [], "left: stop"),
        'S': (right.stop, [], "right: stop"),
        'z': (l_dead_band, [-1.0], "left: decrease dead band"),
        'Z': (r_dead_band, [-1.0], "right: decrease dead band"),
        'x': (l_dead_band, [1.0], "left: increase dead band"),
        'X': (r_dead_band, [1.0], "right: increase dead band"),
        'f': (l_moving, [-5.0], "left: decrease moving force"),
        'F': (r_moving, [-5.0], "right:  decrease moving force"),
        'g': (l_moving, [5.0], "left:  increase moving force"),
        'G': (r_moving, [5.0], "right:  increase moving force"),
        'h': (l_holding, [-5.0], "left:  decrease holding force"),
        'H': (r_holding, [-5.0], "right:  decrease holding force"),
        'j': (l_holding, [5.0], "left:  increase holding force"),
        'J': (r_holding, [5.0], "right:  increase holding force"),
        'v': (l_velocity, [-5.0], "left:  decrease velocity"),
        'V': (l_velocity, [-5.0], "right:  decrease velocity"),
        'b': (l_velocity, [5.0], "left:  increase velocity"),
        'B': (r_velocity, [5.0], "right:  increase velocity"),
        'u': (l_command, [-10.0], "left:  decrease position"),
        'U': (r_command, [-10.0], "right:  decrease position"),
        'i': (l_command, [10.0], "left:  increase position"),
        'I': (r_command, [10.0], "right:  increase position"),
    }

    done = False
    print("Enabling robot... ")
    rs.enable()
    print("Controlling grippers. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_io_devices.getch()
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
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
    # force shutdown call if caught by key handler
    rospy.signal_shutdown("Example finished.")


def main():
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_gripper_keyboard")

    map_keyboard()


if __name__ == '__main__':
    main()
