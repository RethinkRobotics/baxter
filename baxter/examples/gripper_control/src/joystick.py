#!/usr/bin/env python
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
        ((bdn, ['dPadUp']), (left.inc_velocity, [5.0]), "right:  increase velocity"),
        ((bdn, ['function1']), (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']), (print_help, [bindings_list]), "help"),
    )
    bindings_list.append(bindings)

    rate = rospy.Rate(100)
    print("press any key to stop...")
    while not rospy.is_shutdown():
        if iodevices.getch():
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
    parser.add_argument("joystick", help="specify the type of joystick to use; xbox or logitech")
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
    else:
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    if map_joystick(joystick):
        print("Disabling robot... ")
        rs.disable()
        print("done")
    else:
        print("terminated")
