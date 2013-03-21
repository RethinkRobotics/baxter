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


import getopt
import os
import sys

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
    msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
    pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
    pub.publish(msg)
    # Even with the latch, we seem to need to wait a bit before exiting to
    # make sure that the message got sent.  Using a service may be a better
    # idea.
    # See also: http://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/
    rospy.Rate(1).sleep()


def usage():
        print """
%s [ARGUMENTS]

    -h, --help          This screen
    -f, --file [PATH]   Path to image file to send
    """ % (os.path.basename(sys.argv[0]),)

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hf:',
            ['help', 'file=',])
    except getopt.GetoptError as err:
        print str(err)
        usage()
        sys.exit(2)

    path = None
    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif o in ('-f', '--file'):
            path = a

    if not os.access(path, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (path,))
        sys.exit(1)

    rospy.init_node('xdisplay_image', anonymous=True)
    send_image(path)
    sys.exit(0)

if __name__ == '__main__':
    main()
