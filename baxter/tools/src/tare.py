import getopt
import os
import sys

import roslib
roslib.load_manifest('tools')
import rospy

from baxter_msgs.msg import (
    TareEnable,
)

import baxter_interface.robustcontroller

class Tare(baxter_interface.robustcontroller.RobustController):
    def __init__ (self, limb):
        """
        Wrapper to run the Tare RobustController.

        @param limb - Limb to run tare on [left/right]
        """
        enable_msg = TareEnable(isEnabled = True, uid = 'sdk')
        enable_msg.data.tuneGravitySpring = True

        disable_msg = TareEnable(isEnabled = False, uid = 'sdk')

        # Initialize RobustController, use 5 minute timeout for the Tare process
        super(Tare, self).__init__(
            '/sdk/robustcontroller/%s/Tare' % (limb,),
            enable_msg,
            disable_msg,
            5 * 60)

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

    rospy.init_node('tare_sdk', anonymous=True)
    tt = Tare(limb)
    rospy.loginfo("Running tare on %s limb" % (limb,))
    ret, msg = tt.run()

    if ret == 0:
        rospy.loginfo("Tare finished")
    else:
        rospy.logerr("Tare failed: %s" % (msg,))

    sys.exit(ret)

if __name__ == '__main__':
    main()
