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

import baxter_interface.digital_io as DIO


def test_interface(io_component='left_itb_light_outer'):
    """Blinks a Digital Output on then off."""
    rospy.loginfo("Blinking Digital Output: %s", io_component)
    b = DIO.DigitalIO(io_component)

    print "Initial state: ", b.state

    # turn on light
    b.set_output(True)
    rospy.sleep(1)
    print "New state: ", b.state

    # reset output
    b.set_output(False)
    rospy.sleep(1)
    print "Final state:", b.state


def main():
    show_defaults = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=show_defaults)
    parser.add_argument(
        '-c', '--component', dest='component_id',
        default='left_itb_light_outer',
        help='name of Digital IO component to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('test_dio', anonymous=True)
    io_component = rospy.get_param('~component_id', args.component_id)
    test_interface(io_component)

if __name__ == '__main__':
    main()
