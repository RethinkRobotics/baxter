#!/usr/bin/env python
"""
Baxter RSDK Joint Position Example: file playback
"""
import argparse
import sys

import roslib
roslib.load_manifest('joint_position')
import rospy

import baxter_interface
import iodevices

def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None

def map_file(filename, loops=1):
    """ Loops through csv file
    Does not loop indefinitely, but only until the file is read
    and processed. Reads each line, split up in columns and
    formats each line into a controller command in the form of
    name/value pairs. Names come from the column headers
    first column is the time stamp
    @param filename - the file to play
    @param loops - number of times to loop
        values < 0 mean 'infinite'
    """
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left')
    grip_right = baxter_interface.Gripper('right')
    rate = rospy.Rate(1000)
    start_time = rospy.get_time()
    print("playing back %s" % (filename,))
    with open(filename, 'r') as f:
        lines = f.readlines()
    keys = lines[0].rstrip().split(',')
    i = 0
    l = 0
    while loops < 1 or l < loops:
        l = l+1
        for values in lines[1:]:
            i = i +1
            loopstr = str(loops) if loops > 0 else "forever"
            sys.stdout.write("\r record %d of %d, loop %d of %s" % (i, len(lines),l,loopstr))
            sys.stdout.flush()

            #convert the line of strings to a float or None
            values = [try_float(x) for x in values.rstrip().split(',')]
            #zip the values with the joint names
            combined = zip(keys[1:], values[1:])
            #take out any tuples that have a none value
            cleaned = [x for x in combined if x[1] is not None]
            #convert it to a dictionary with only valid commands
            cmd = dict(cleaned)
            lcmd = dict((key[-2:], cmd[key]) for key in cmd.keys() if key[:-2] == 'left_')
            rcmd = dict((key[-2:], cmd[key]) for key in cmd.keys() if key[:-2] == 'right_')

            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < values[0]:
                if rospy.is_shutdown():
                    print("\n ROS shutdown")
                    return False
                if iodevices.getch():
                    print("\n stopped")
                    return False
                if len(lcmd):
                    left.set_positions(lcmd)
                if len(rcmd):
                    right.set_positions(rcmd)
                if 'left_gripper' in cmd:
                    grip_left.set_position(cmd['left_gripper'])
                if 'left_gripper' in cmd:
                    grip_right.set_position(cmd['right_gripper'])
                rate.sleep()
    print
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="input file")
    parser.add_argument("-l", "--loops", default=1, help="number of times to loop the input file. 0=infinite.")
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_position_file_playback")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    map_file(args.file, int(args.loops))
    rs.disable()
