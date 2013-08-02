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
roslib.load_manifest('input_output')
import rospy

import baxter_interface

def usage():
    print """
%s [ARGUMENTS]

    -h, --help          This screen
    -b, --blink         Blink navigator lights for 10 seconds
    -i, --input         Show input for 10 seconds
    """ % (os.path.basename(sys.argv[0]),)

def blink():
    navs = (
        baxter_interface.Navigator('left'),
        baxter_interface.Navigator('right'),
        baxter_interface.Navigator('torso_left'),
        baxter_interface.Navigator('torso_right'),)

    print ("Blinking LED's for 10 seconds")
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown() and i < 100:
        for nav in navs:
            nav.inner_led = not nav.inner_led
            nav.outer_led = not nav.outer_led
        rate.sleep()
        i += 1

def echo_input():
    def b0_pressed(v):
        print ("Button 0: %s" % (v,))
    def b1_pressed(v):
        print ("Button 1: %s" % (v,))
    def b2_pressed(v):
        print ("Button 2: %s" % (v,))
    def wheel_moved(v):
        print ("Wheel: %d" % (v,))

    print ("Press input buttons on the left navigator, input will be echoed here.")
    nav = baxter_interface.Navigator('left')
    nav.button0_changed.connect(b0_pressed)
    nav.button1_changed.connect(b1_pressed)
    nav.button2_changed.connect(b2_pressed)
    nav.wheel_changed.connect(wheel_moved)

    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown() and i < 10:
        rate.sleep()
        i += 1

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hbi',
            ['help', 'blink', 'input',])
    except getopt.GetoptError as err:
        print str(err)
        usage()
        sys.exit(2)

    action = None

    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif o in ('-b', '--blink'):
            action = blink
        elif o in ('-i', '--input'):
            action = echo_input

    if action == None:
        print ("No action defined")
        usage()
        sys.exit(2)

    rospy.init_node('navigator_example', anonymous = True)
    action()
    sys.exit(0)

if __name__ == '__main__':
    main()

