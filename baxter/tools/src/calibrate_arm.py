#!/usr/bin/python2

import getopt
import os
import sys

import roslib
roslib.load_manifest('tools')
import rospy

from baxter_msgs.msg import (
    CalibrateArmEnable,
)

import baxter_interface
import baxter_interface.robustcontroller

class CalibrateArm(baxter_interface.robustcontroller.RobustController):
    def __init__ (self, limb):
        """
        Wrapper to run the CalibrateArm RobustController.

        @param limb - Limb to run CalibrateArm on [left/right]
        """
        enable_msg = CalibrateArmEnable(isEnabled = True, uid = 'sdk')

        disable_msg = CalibrateArmEnable(isEnabled = False, uid = 'sdk')

        # Initialize RobustController, use 10 minute timeout for the CalibrateArm process
        super(CalibrateArm, self).__init__(
            '/sdk/robustcontroller/%s/CalibrateArm' % (limb,),
            enable_msg,
            disable_msg,
            10 * 60)

def usage():
        print """
%s [ARGUMENTS]

    -h, --help              This screen
    -c, --calibrate [LIMB]  Calibrate the specified arm [left/right]
    """ % (os.path.basename(sys.argv[0]),)

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hc:',
            ['help', 'calibrate=',])
    except getopt.GetoptError as err:
        print str(err)
        usage()
        sys.exit(2)

    arm = None
    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif o in ('-c', '--calibrate'):
            arm = a

    if not arm:
        usage()
        rospy.logerr("No arm specified")
        sys.exit(1)

    rospy.init_node('calibrate_arm_sdk', anonymous=True)
    rs = baxter_interface.RobotEnable()
    rs.enable()
    cat = CalibrateArm(arm)
    rospy.loginfo("Running calibrate on %s arm" % (arm,))

    error = None
    try:
        cat.run()
    except IOError, e:
        error = e.strerror
    else:
        rs.disable()

    if error == None:
        rospy.loginfo("Calibrate arm finished")
    else:
        rospy.logerr("Calibrate arm failed: %s" % (error,))

    sys.exit(0 if error == None else 1)

if __name__ == '__main__':
    main()
