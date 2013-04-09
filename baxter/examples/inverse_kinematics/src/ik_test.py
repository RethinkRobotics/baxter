#!/usr/bin/env python

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

"""
Baxter RSDK Inverse Kinematics Example:
 A simple python example of using the Rethink IK
 to get the joint angles for a Cartesian Pose or
 to just check if a pose is valid/possible.

"""
import argparse
import sys

import roslib; roslib.load_manifest('inverse_kinematics')
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_msgs.srv import SolvePositionIK
from baxter_msgs.srv import SolvePositionIKRequest

"""
def talker():
    # declare shared endpoint data
    global endpoint;

    # select which arm to use
    limb = 'right'

    # a variable to mimic Rethink "travel height", some height above current endpt state
    travel_height = 0.18

    # wait for the IK service to be available
    rospy.wait_for_service("/ExternalTools/" + limb + "/PositionIKNode/IKService")

    # create a service caller object
    computeIK = rospy.ServiceProxy("/ExternalTools/" + limb + "/PositionIKNode/IKService",
                                    SolvePositionIK)

    rospy.init_node('talker')

    # allocate space for the IK request
    ikref = SolvePositionIKRequest()
    ikref.pose_stamp.append(PoseStamped())
    ikref.pose_stamp.append(PoseStamped())

    # loop until ctrl-c
    while not rospy.is_shutdown():
        # fill in the IK request
        ikref.pose_stamp[0].header.seq = 0
        ikref.pose_stamp[0].header.stamp = rospy.Time.now()
        ikref.pose_stamp[0].header.frame_id = 'base'
        # copy over the current endpoint pose as the pose we'd like to to IK at
        ikref.pose_stamp[0].pose = copy.deepcopy(endpoint.pose)

        # check a new pose that is translated up to "travel height"
        ikref.pose_stamp[1] = copy.deepcopy(ikref.pose_stamp[0])
        ikref.pose_stamp[1].pose.position.z =  ikref.pose_stamp[1].pose.position.z + travel_height

        start = rospy.get_rostime()
        # compute IK
        try:
          resp = computeIK(ikref)
        except rospy.ServiceException,e :
          rospy.loginfo("Service call failed: %s"%e)
        stop = rospy.get_rostime()

        # output to the user if we were successful or not
        if resp.isValid[0]:
          rospy.loginfo("IK found valid result for current pose.")
        else:
          rospy.loginfo("Current pose invalid.")
        if resp.isValid[1]:
          rospy.loginfo("IK found valid result for travel pose.")
        else:
          rospy.loginfo("Travel height invalid.")
        rospy.sleep(1.0)

        rospy.loginfo("IK call took %f seconds." % (stop-start).to_sec())
"""

def main(limb):
    rospy.init_node("rethink_rsdk_inverse_kinematics_test")
    ns = "/sdk/robot/limb/" + limb + "/solve_ik_position"
    rospy.wait_for_service(ns)
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.657579481614,
                    y=0.851981417433,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        resp = iksvc(ikreq)
    except rospy.ServiceException,e :
        rospy.loginfo("Service call failed: %s" % (e,))
    print(resp)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("limb", help="the limb to test <left | right>")
    args = parser.parse_args()

    main(args.limb)
