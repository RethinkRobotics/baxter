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

import argparse

import rospy

import baxter_interface.analog_io as AIO


def test_interface(io_component='torso_fan'):
    """Ramps an Analog component from 0 to 100, then back down to 0."""
    rospy.loginfo("Ramping output of Analog IO component: %s", io_component)

    b = AIO.AnalogIO(io_component)
    rate = rospy.Rate(2)

    # start: 0.0
    print b.state()

    # ramp up
    for i in range(0, 101, 10):
        b.set_output(i)
        print i
        rate.sleep()
    # max: 100.0
    print b.state()

    # ramp down
    for i in range(100, -1, -10):
        b.set_output(i)
        print i
        rate.sleep()
    # (fans off)
    b.set_output(0)


def main():
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        '-c', '--component', dest='component_id', default='torso_fan',
        help='name of Analog IO component to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('test_aio', anonymous=True)
    io_component = rospy.get_param('~component_id', args.component_id)
    test_interface(io_component)

if __name__ == '__main__':
    main()
