#!/usr/bin/python2
import errno
import getopt
import os
import sys
import time

import roslib
roslib.load_manifest('robot_control')
import baxter_msgs.msg
import baxter_msgs.srv
import rospy

class RobotState(object):
    """
    Class RobotState - simple control/status wrapper around robot state

    enable()    - enable all joints
    disable()   - disable all joints
    reset()     - reset all joints, reset all jrcp faults, disable the robot
    stop()      - stop the robot, similar to hitting the e-stop button

    """
    def __init__(self):
        self._initialized = False
        self._state = None
        self._state_sub = rospy.Subscriber(
            '/sdk/robot/state',
            baxter_msgs.msg.AssemblyState,
            self._state_callback)
        self._get_state()

    def _state_callback(self, msg):
        self._initialized = True
        self._state = msg

    def _get_state(self):
        self._initialized = False

        timeout = 10
        waited = 0
        while not self._initialized and waited < timeout:
            time.sleep(0.25)
            waited += 0.25

        if not self._initialized:
            raise IOError(errno.ETIMEDOUT, "Failed to get current robot state from /sdk/robot/state")

    def state(self):
        """
        Returns the last known robot state.
        """
        return self._state

    def enable(self):
        """
        Enable all joints
        """
        if self._state.stopped:
            raise IOError(errno.EINVAL, "Cannot enabled a robot that is in the 'stopped' state.  Reset it first.")
        self._toggle_enabled(True)

    def disable(self):
        """
        Disable all joints
        """
        self._toggle_enabled(False)

    def _toggle_enabled(self, status):
        if self._state.enabled == status:
            return

        sname = '/sdk/robot/set_super_enable'
        rospy.wait_for_service(sname)

        svc = rospy.ServiceProxy(sname, baxter_msgs.srv.SetSupervisorEnable)

        try:
            resp = svc(status)
        except rospy.ServiceException, e:
            raise IOError(errno.EIO, "Could not call service %s (%s)" % (sname, str(status)))

        if resp.is_enabled != status:
            raise IOError(errno.EREMOTEIO, "Failed to set %sable robot" % ("en" if status else "dis",))

    def reset(self):
        """
        Reset all joints.  Trigger JRCP hardware to reset all faults.  Disable
        the robot.
        """
        sname = '/sdk/robot/set_super_reset'
        rospy.wait_for_service(sname)
        svc = rospy.ServiceProxy(sname, baxter_msgs.srv.SetSupervisorReset)

        try:
            resp = svc(True)
        except rospy.ServiceException, e:
            raise IOError(errno.EIO, "Could not call service %s (True)" % (sname,))

        if not resp.is_reset:
            raise IOError(errno.EREMOTEIO, "Failed to reset robot")

    def stop(self):
        """
        Simulate an e-stop button being pressed.  Robot must be reset to clear
        the stopped state.
        """
        sname = '/sdk/robot/set_super_stop'
        rospy.wait_for_service(sname)
        svc = rospy.ServiceProxy(sname, baxter_msgs.srv.SetSupervisorStop)

        try:
            resp = svc(True)
        except rospy.ServiceException, e:
            raise IOError(errno.EIO, "Could not call service %s (True)" % (sname,))

        if not resp.is_stopping:
            raise IOError(errno.EREMOTEIO, "Failed to stop robot")



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
            rs = RobotState()

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

