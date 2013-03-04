import errno

import roslib
roslib.load_manifest('baxter_interface')
import rospy

from baxter_msgs.msg import (
    RobustControllerStatus,
)

class RobustController(object):
    STATE_IDLE = 0
    STATE_STARTING = 1
    STATE_RUNNING = 2
    STATE_STOPPING = 3

    def __init__(self, namespace, enable_msg, disable_msg, timeout = 60):
        """
        Wrapper around controlling a RobustController

        @param namespace    - namespace containing the enable and status topics
        @param enable_msg   - message to send to enable the RC
        @param disable_msg  - message to send to disable the RC
        @param timeout      - seconds to wait for the RC to finish [60]
        """
        self._command_pub = rospy.Publisher(
            namespace + '/enable',
            type(enable_msg))
        self._status_sub = rospy.Subscriber(
            namespace + '/status',
            RobustControllerStatus,
            self._callback)

        self._enable_msg = enable_msg
        self._disable_msg = disable_msg
        self._timeout = timeout
        self._state = self.STATE_IDLE
        self._return = 0

        rospy.on_shutdown(self._on_shutdown)

    def _callback(self, msg):
        if self._state == self.STATE_RUNNING:
            if msg.complete == RobustControllerStatus.COMPLETE_W_SUCCESS:
                self._state = self.STATE_STOPPING
                self._return = 0

            elif msg.complete == RobustControllerStatus.COMPLETE_W_FAILURE:
                self._state = self.STATE_STOPPING
                self._return = errno.EIO

            elif not msg.isEnabled:
                self._state = self.STATE_IDLE
                self._return = errno.ENOMSG

        elif self._state == self.STATE_STOPPING and not msg.isEnabled:
            # Would be nice to use msg.state here, but it does not
            # consistently reflect reality.
            self._state = self.STATE_IDLE

        elif self._state == self.STATE_STARTING and msg.isEnabled:
            self._state = self.STATE_RUNNING

    def _run_loop(self):
        # RobustControllers need messages at < 1Hz in order to continue
        # their current operation.
        rate = rospy.Rate(2)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            if self._state == self.STATE_RUNNING and (rospy.Time.now() - start).to_sec() > self._timeout:
                self._state = self.STATE_STOPPING
                self._command_pub.publish(self._disable_msg)
                self._return = errno.ETIMEDOUT

            elif self._state in (self.STATE_STARTING, self.STATE_RUNNING):
                self._command_pub.publish(self._enable_msg)

            elif self._state == self.STATE_STOPPING:
                self._command_pub.publish(self._disable_msg)

            elif self._state == self.STATE_IDLE:
                break

            rate.sleep()

    def _on_shutdown(self):
        rate = rospy.Rate(2)

        while not self._state == self.STATE_IDLE:
            self._command_pub.publish(self._disable_msg)
            rate.sleep()

        self._return = errno.ECONNABORTED

    def run(self):
        """
        Enable the RobustController and run until completion or error.
        """
        self._state = self.STATE_STARTING
        self._command_pub.publish(self._enable_msg)
        self._run_loop()
        if self._return != 0:
            if self._return == errno.EIO:
                raise IOError(self._return, "Robust controller failed")
            elif self._return == errno.ENOMSG:
                raise IOError(self._return, "Robust controller failed to enable")
            elif self._return == errno.ETIMEDOUT:
                raise IOError(self._return, "Robust controller timed out")
            elif self._return == errno.ECONNABORTED:
                raise IOError(self._return, "Robust controller interruped by user")
            else:
                raise IOError(self._return)

