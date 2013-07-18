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

def clean_line(line, names):
    """ Cleans a single line of recorded joint positions
    @param line - the line described in a list to process
    @param names - joint name keys
    """
    #convert the line of strings to a float or None
    line = [try_float(x) for x in line.rstrip().split(',')]
    #zip the values with the joint names
    combined = zip(names[1:], line[1:])
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    left_command = dict((key, command[key]) for key in command.keys() \
        if key[:-2] == 'left_')
    right_command = dict((key, command[key]) for key in command.keys() \
        if key[:-2] == 'right_')
    return (command, left_command, right_command, line)

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
    print("Playing back: %s" % (filename,))
    with open(filename, 'r') as f:
        lines = f.readlines()
    keys = lines[0].rstrip().split(',')
    i = 0
    l = 0
    # If specified, repeat the file playback 'loops' number of times
    while loops < 1 or l < loops:
        l = l+1
        print("Moving to start position...")
        cmd_start, lcmd_start, rcmd_start, raw_start = clean_line(lines[1], keys)
        left.move_to_joint_positions(lcmd_start)
        right.move_to_joint_positions(rcmd_start)
        for values in lines[1:]:
            i = i +1
            loopstr = str(loops) if loops > 0 else "forever"
            sys.stdout.write("\r Record %d of %d, loop %d of %s" \
                % (i, len(lines)-1,l,loopstr))
            sys.stdout.flush()

            cmd, lcmd, rcmd, values = clean_line(values, keys)
            #command this set of commands until the next frame
            while (rospy.get_time() - start_time) < values[0]:
                if rospy.is_shutdown():
                    print("\n ROS shutdown")
                    return False
                if iodevices.getch():
                    print("\n stopped")
                    return False
                if len(lcmd):
                    left.set_joint_positions(lcmd)
                if len(rcmd):
                    right.set_joint_positions(rcmd)
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
    parser.add_argument("-l", "--loops", type=int, default=1, \
        help="number of times to loop the input file. 0=infinite.")
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_position_file_playback")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    map_file(args.file, args.loops)
