import getopt
import os
import sys

import roslib
roslib.load_manifest('calibration')
import rospy

from baxter_msgs.msg import (
    CalibrateArmEnable,
)

import baxter_interface.robustcontroller

class CalibrateArmTester(baxter_interface.robustcontroller.RobustController):
    def __init__ (self, limb):
        """
        Wrapper to run the CalibrateArm RobustController.

        @param limb - Limb to run CalibrateArm on [left/right]
        """
        # Enable message
        e = CalibrateArmEnable()
        e.isEnabled = True
        e.uid = 'tester'

        # Disable Message
        d = CalibrateArmEnable()
        d.isEnabled = False
        d.uid = 'tester'

        # Initialize RobustController, use 10 minute timeout for the CalibrateArm process
        super(CalibrateArmTester, self).__init__('/robustcontroller/%s/CalibrateArm' % (limb,), e, d, 10 * 60)

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

    rospy.init_node('calibrate_arm_tester', anonymous=True)
    cat = CalibrateArmTester(arm)
    rospy.loginfo("Running calibrate on %s arm" % (arm,))
    ret, msg = cat.run()

    if not ret:
        rospy.loginfo("Calibrate arm finished")
    else:
        rospy.logerr("Calibrate arm failed: %s" % (msg,))

    sys.exit(ret)

if __name__ == '__main__':
    main()
