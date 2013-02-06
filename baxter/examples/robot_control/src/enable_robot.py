#!/usr/bin/python2
import getopt
import os
import sys

import roslib
roslib.load_manifest('robot_control')
import rospy

import baxter_interface

def usage():
    print """
%s [ARGUMENTS]

    -h, --help      This screen
    -s, --state     Print current robot state

    -e, --enable    Enable the robot
    -d, --disable   Disable the robot
    -r, --reset     Reset the robot
    -S, --stop      Stop the robot
    """ % (os.path.basename(sys.argv[0]),)

if __name__ == '__main__':

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hsedrS',
            ['help', 'state', 'enable', 'disable', 'reset', 'stop'])
    except getopt.GetoptError as err:
        print str(err)
        usage()
        sys.exit(2)

    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            sys.exit(0)
        else:
            rospy.init_node('robot_control')
            rs = baxter_interface.RobotEnable()

            if o in ('-s', '--state'):
                print rs.state()
            elif o in ('-e', '--enable'):
                rs.enable()
            elif o in ('-d', '--disable'):
                rs.disable()
            elif o in ('-r', '--reset'):
                rs.reset()
            elif o in ('-S', '--stop'):
                rs.stop()

    sys.exit(0)

