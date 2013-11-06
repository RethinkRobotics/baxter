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

import rospy

from std_msgs.msg import (
    Bool
)

import baxter_dataflow

from baxter_core_msgs.msg import (
   HeadPanCommand,
   HeadState,
)
from baxter_interface import settings


class Head(object):
    def __init__(self):
        """
        Interface class for the head on the Baxter Robot.

        Used to control the head pan angle and to enable/disable the head nod
        action.
        """
        self._state = {}

        self._pub_pan = rospy.Publisher(
            '/robot/head/command_head_pan',
            HeadPanCommand)

        self._pub_nod = rospy.Publisher(
            '/robot/head/command_head_nod',
            Bool)

        state_topic = '/robot/head/head_state'
        self._sub_state = rospy.Subscriber(
            state_topic,
            HeadState,
            self._on_head_state)

        baxter_dataflow.wait_for(
            lambda: len(self._state) != 0,
            timeout=5.0,
            timeout_msg=("Failed to get current head state from %s" %
                         (state_topic,)),
        )

    def _on_head_state(self, msg):
        self._state['pan'] = msg.pan
        self._state['panning'] = msg.isPanning
        self._state['nodding'] = msg.isNodding

    def pan(self):
        """
        Get the current pan angle of the head.

        @returns (float)    - current angle in radians
        """
        return self._state['pan']

    def nodding(self):
        """
        Check if the head is currently nodding.

        @returns (bool) - True if the head is currently nodding, False
                          otherwise.
        """
        return self._state['nodding']

    def panning(self):
        """
        Check if the head is currently panning.

        @returns (bool) - True if the head is currently panning, False
                          otherwise.
        """
        return self._state['panning']

    def set_pan(self, angle, speed=100, timeout=10.0):
        """
        @param angle (float)   - Desired pan angle in radians.
        @param speed (int)     - Desired speed to pan at, range is 0-100 [100]
        @param timeout (float) - Seconds to wait for the head to pan to the
                                 specified angle. If 0, just command once and
                                 return. [10]

        Pan at the given speed to the desired angle.
        """
        msg = HeadPanCommand(angle, speed)
        self._pub_pan.publish(msg)

        if not timeout == 0:
            baxter_dataflow.wait_for(
                lambda: (abs(self.pan() - angle) <=
                         settings.HEAD_PAN_ANGLE_TOLERANCE),
                timeout=timeout,
                rate=100,
                timeout_msg="Failed to move head to pan command %f" % angle,
                body=lambda: self._pub_pan.publish(msg)
                )

    def command_nod(self, timeout=5.0):
        """
        @param timeout (float)  - Seconds to wait for the head to nod.
                                  If 0, just command once and return.  [0]

        Command the head to nod once.
        """
        self._pub_nod.publish(True)

        if not timeout == 0:
            # Wait for nod to initiate
            baxter_dataflow.wait_for(
                test=self.nodding,
                timeout=timeout,
                rate=100,
                timeout_msg="Failed to initiate head nod command",
                body=lambda: self._pub_nod.publish(True)
            )

            # Wait for nod to complete
            baxter_dataflow.wait_for(
                test=lambda: not self.nodding(),
                timeout=timeout,
                rate=100,
                timeout_msg="Failed to complete head nod command",
                body=lambda: self._pub_nod.publish(True)
            )
