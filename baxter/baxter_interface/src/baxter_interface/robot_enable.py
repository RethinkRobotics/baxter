import errno
import os
import sys
import time

import roslib
roslib.load_manifest('baxter_interface')
import rospy
import baxter_msgs.msg
import baxter_msgs.srv

class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

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


