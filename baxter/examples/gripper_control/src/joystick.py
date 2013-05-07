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
Baxter RSDK Gripper Example: joystick
"""
import argparse

import roslib
roslib.load_manifest('gripper_control')
import rospy

import baxter_interface
import iodevices

def map_joystick(joystick):
    """
    maps joystick input to gripper commands
    @param joystick - an instance of a Joystick
    """
    left = baxter_interface.Gripper('left')
    right = baxter_interface.Gripper('right')

    #abbreviations
    jhi = lambda s: joystick.stick_value(s) > 0
    jlo = lambda s: joystick.stick_value(s) < 0
    bdn = joystick.button_down
    bup = joystick.button_up

    def print_help(bindings_list):
        print("press any keyboard key to quit.")
        for bindings in bindings_list:
            for (test, cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s %s: %s" % (test[0].__name__, str(test[1]), doc))

    bindings_list = []
    bindings = (
        ((bdn, ['btnDown']), (left.reboot, []), "left: reset"),
        ((bdn, ['btnLeft']), (right.reboot, []), "right: reset"),
        ((bdn, ['btnRight']), (left.calibrate, []), "left: calibrate"),
        ((bdn, ['btnUp']), (right.calibrate, []), "right: calibrate"),
        ((bdn, ['rightTrigger']), (left.close, []), "left: close"),
        ((bdn, ['leftTrigger']), (right.close, []), "right: close"),
        ((bup, ['rightTrigger']), (left.open, []), "left: open"),
        ((bup, ['leftTrigger']), (right.open, []), "right: open"),
        ((bdn, ['rightBumper']), (left.stop, []), "left: stop"),
        ((bdn, ['leftBumper']), (right.stop, []), "right: stop"),
        ((jlo, ['rightStickHorz']), (left.inc_position, [-5.0]), "left:  decrease position"),
        ((jlo, ['leftStickHorz']), (right.inc_position, [-5.0]), "right:  decrease position"),
        ((jhi, ['rightStickHorz']), (left.inc_position, [5.0]), "left:  increase position"),
        ((jhi, ['leftStickHorz']), (right.inc_position, [5.0]), "right:  increase position"),
        ((jlo, ['rightStickVert']), (left.inc_holding_force, [-5.0]), "left:  decrease holding force"),
        ((jlo, ['leftStickVert']), (right.inc_holding_force, [-5.0]), "right:  decrease holding force"),
        ((jhi, ['rightStickVert']), (left.inc_holding_force, [5.0]), "left:  increase holding force"),
        ((jhi, ['leftStickVert']), (right.inc_holding_force, [5.0]), "right:  increase holding force"),
        ((bdn, ['dPadDown']), (left.inc_velocity, [-5.0]), "left:  decrease velocity"),
        ((bdn, ['dPadLeft']), (right.inc_velocity, [-5.0]), "right:  decrease velocity"),
        ((bdn, ['dPadRight']), (left.inc_velocity, [5.0]), "left:  increase velocity"),
        ((bdn, ['dPadUp']), (right.inc_velocity, [5.0]), "right:  increase velocity"),
        ((bdn, ['function1']), (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']), (print_help, [bindings_list]), "help"),
    )
    bindings_list.append(bindings)

    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("press any key to stop...")
    while not rospy.is_shutdown():
        c = iodevices.getch()
        if c:
            if c == '?':
                print_help(bindings_list)
            else:
                return True
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)
        rate.sleep()
    return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("joystick", help="specify the type of joystick to use; xbox | logitech | ps3")
    args, unknown = parser.parse_known_args()

    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_gripper_control_joystick")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    joystick = None
    if args.joystick == 'xbox':
        joystick = iodevices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = iodevices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = iodevices.joystick.PS3Controller()
    else:
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    if map_joystick(joystick):
        print("Disabling robot... ")
        rs.disable()
        print("done")
    else:
        print("terminated")
