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

import rospy

import baxter_interface
import baxter_io_devices


def map_joystick(joystick):
    """
    maps joystick input to gripper commands
    @param joystick - an instance of a Joystick
    """
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

    # abbreviations
    jhi = lambda s: joystick.stick_value(s) > 0
    jlo = lambda s: joystick.stick_value(s) < 0
    bdn = joystick.button_down
    bup = joystick.button_up

    def print_help(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s %s: %s" % (test[0].__name__, str(test[1]), doc))

    def l_command(offset):
        left.command_position(left.position() + offset)

    def r_command(offset):
        right.command_position(right.position() + offset)

    def l_holding(offset):
        left.set_holding_force(left.parameters()['holding_force'] + offset)

    def r_holding(offset):
        right.set_holding_force(right.parameters()['holding_force'] + offset)

    def l_velocity(offset):
        left.set_velocity(left.parameters()['velocity'] + offset)

    def r_velocity(offset):
        right.set_velocity(right.parameters()['velocity'] + offset)

    bindings_list = []
    bindings = (
        #(test, command, description)
        ((bdn, ['btnDown']), (left.reboot, []), "left: reboot"),
        ((bdn, ['btnLeft']), (right.reboot, []), "right: reboot"),
        ((bdn, ['btnRight']), (left.calibrate, []), "left: calibrate"),
        ((bdn, ['btnUp']), (right.calibrate, []), "right: calibrate"),
        ((bdn, ['rightTrigger']), (left.close, []), "left: close"),
        ((bdn, ['leftTrigger']), (right.close, []), "right: close"),
        ((bup, ['rightTrigger']), (left.open, []), "left: open"),
        ((bup, ['leftTrigger']), (right.open, []), "right: open"),
        ((bdn, ['rightBumper']), (left.stop, []), "left: stop"),
        ((bdn, ['leftBumper']), (right.stop, []), "right: stop"),
        ((jlo, ['rightStickHorz']), (l_command, [-7.5]),
                                     "left:  decrease position"),
        ((jlo, ['leftStickHorz']), (r_command, [-7.5]),
                                    "right:  decrease position"),
        ((jhi, ['rightStickHorz']), (l_command, [7.5]),
                                     "left:  increase position"),
        ((jhi, ['leftStickHorz']), (r_command, [7.5]),
                                     "right:  increase position"),
        ((jlo, ['rightStickVert']), (l_holding, [-5.0]),
                                     "left:  decrease holding force"),
        ((jlo, ['leftStickVert']), (r_holding, [-5.0]),
                                    "right:  decrease holding force"),
        ((jhi, ['rightStickVert']), (l_holding, [5.0]),
                                     "left:  increase holding force"),
        ((jhi, ['leftStickVert']), (r_holding, [5.0]),
                                    "right:  increase holding force"),
        ((bdn, ['dPadDown']), (l_velocity, [-5.0]),
                               "left:  decrease velocity"),
        ((bdn, ['dPadLeft']), (r_velocity, [-5.0]),
                               "right:  decrease velocity"),
        ((bdn, ['dPadRight']), (l_velocity, [5.0]),
                                "left:  increase velocity"),
        ((bdn, ['dPadUp']), (r_velocity, [5.0]),
                             "right:  increase velocity"),
        ((bdn, ['function1']), (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']), (print_help, [bindings_list]), "help"),
    )
    bindings_list.append(bindings)

    print("Enabling robot...")
    rs.enable()
    rate = rospy.Rate(100)
    print_help(bindings_list)
    print("Press <Start> button for help; Ctrl-C to stop...")
    while not rospy.is_shutdown():
        # test each joystick condition and call binding cmd if true
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)
        rate.sleep()
    rospy.signal_shutdown("Example finished.")


def main():
    parser = argparse.ArgumentParser()
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--joystick', required=True, choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    joystick = None
    if args.joystick == 'xbox':
        joystick = baxter_io_devices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = baxter_io_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = baxter_io_devices.joystick.PS3Controller()
    else:
        # Should never reach this case with proper argparse usage
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_gripper_control_joystick")

    map_joystick(joystick)


if __name__ == '__main__':
    main()
