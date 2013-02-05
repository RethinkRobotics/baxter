import getopt
import os
import sys

import roslib
roslib.load_manifest('calibration')
import rospy

from baxter_msgs.msg import (
    TareEnable,
)

import baxter_interface.robustcontroller

class TareTester(baxter_interface.robustcontroller.RobustController):
    def __init__ (self, limb):
        """
        Wrapper to run the Tare RobustController.

        @param limb - Limb to run tare on [left/right]
        """
        # Enable message
        e = TareEnable()
        e.data.tuneGravitySpring = True
        e.isEnabled = True
        e.uid = 'tester'

        # Disable Message
        d = TareEnable()
        d.isEnabled = False
        d.uid = 'tester'

        # Initialize RobustController, use 5 minute timeout for the Tare process
        super(TareTester, self).__init__('/sdk/robustcontroller/%s/Tare' % (limb,), e, d, 5 * 60)

def usage():
        print """
%s [ARGUMENTS]

    -h, --help          This screen
    -t, --tare [LIMB]   Tare the specified limb [left/right]
    """ % (os.path.basename(sys.argv[0]),)

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'ht:',
            ['help', 'tare=',])
    except getopt.GetoptError as err:
        print str(err)
        usage()
        sys.exit(2)

    limb = None
    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif o in ('-t', '--tare'):
            limb = a

    if not limb:
        usage()
        rospy.logerr("No limb specified")
        sys.exit(1)

    rospy.init_node('tare_tester', anonymous=True)
    tt = TareTester(limb)
    rospy.loginfo("Running tare on %s limb" % (limb,))
    ret, msg = tt.run()

    if not ret:
        rospy.loginfo("Tare finished")
    else:
        rospy.logerr("Tare failed: %s" % (msg,))

    sys.exit(ret)

if __name__ == '__main__':
    main()
