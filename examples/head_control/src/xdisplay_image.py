#!/usr/bin/python2

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


import os
import sys
import argparse

import roslib
roslib.load_manifest('head_control')
import rospy

import cv
import cv_bridge

import sensor_msgs.msg

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path - path to the image file to load and send
    """
    img = cv.LoadImage(path)
    msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', sensor_msgs.msg.Image, latch=True)
    pub.publish(msg)
    # Even with latching, we seem to need to wait a bit before exiting to make
    # sure that the message got sent.  Using a service may be a better idea.
    # See also: http://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/
    rospy.sleep(1)

def main(path=None, delay=0.0):
    rospy.init_node('xdisplay_image', anonymous=True)

    if not os.access(path, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (path,))
        sys.exit(1)

    # Wait for specified time
    if delay > 0:
        rospy.loginfo("Waiting for %s seconds before publishing image to face" % (delay,))
        rospy.sleep(delay)

    send_image(path)
    sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    required = parser.add_argument_group('required arguments')
    required.add_argument('-f', '--file', metavar='PATH', required=True,
                  help='Path to image file to send')
    parser.add_argument('-d', '--delay', metavar='SEC', type=float,
                  default=0.0,
                  help='Time in seconds to wait before publishing image')
    args = parser.parse_args(rospy.myargv()[1:])

    main(args.file, args.delay)
