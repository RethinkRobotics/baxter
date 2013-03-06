#!/usr/bin/env python
import signal
import math
import random

import roslib
roslib.load_manifest('head_control')
import rospy
import baxter_interface

class Wobbler():

    def __init__(self):
        """
        'Wobbles' the head

        """
        self._done = False
        signal.signal(signal.SIGINT, self._handle_ctrl_c)
        self._head = baxter_interface.Head()

    def _handle_ctrl_c(self, signum, frame):
       print("stopping...")
       self._done = True

    def set_neutral(self):
        """
        Sets the head back into a neutral pose

        """
        self._head.set_pan(0.0)

    def wobble(self):
        self.set_neutral()
        """
        Performs the wobbling

        """
        self._head.command_nod()
        rate = rospy.Rate(1);
        start = rospy.get_time()
        while (rospy.get_time() - start < 5.0):
            angle = random.uniform(-1.5, 1.5)
            self._head.set_pan(angle)

        #return to normal
        self.set_neutral()

if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("rethink_rsdk_joint_velocity")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()

    wobbler = Wobbler()
    print("Wobbling... ")
    wobbler.wobble()

    print("Disabling robot... ")
    rs.disable()
    print("done.")
