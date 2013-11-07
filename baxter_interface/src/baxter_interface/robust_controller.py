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

import errno

import rospy

from baxter_core_msgs.msg import (
    RobustControllerStatus,
)


class RobustController(object):
    (STATE_IDLE,
     STATE_STARTING,
     STATE_RUNNING,
     STATE_STOPPING) = range(4)

    def __init__(self, namespace, enable_msg, disable_msg, timeout=60):
        """
        @param namespace   - namespace containing the enable and status topics
        @param enable_msg  - message to send to enable the RC
        @param disable_msg - message to send to disable the RC
        @param timeout     - seconds to wait for the RC to finish [60]

        Wrapper around controlling a RobustController
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
            if (self._state == self.STATE_RUNNING and
                (rospy.Time.now() - start).to_sec() > self._timeout):
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
            msgs = {
                errno.EIO:          "Robust controller failed",
                errno.ENOMSG:       "Robust controller failed to enable",
                errno.ETIMEDOUT:    "Robust controller timed out",
                errno.ECONNABORTED: "Robust controller interrupted by user",
                }

            msg = msgs.get(self._return, None)
            if msg:
                raise IOError(self._return, msg)
            raise IOError(self._return)
