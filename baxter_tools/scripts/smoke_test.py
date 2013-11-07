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
RSDK Smoke Test Execution
"""
import sys
import time
import argparse
import re
import socket
import traceback

import rospy

from baxter_tools import smoketests


def run_test(tname, fname, proceed):
    """
    Execution of the tests where starting, finishing and error handeling
    occurs.
    """
    try:
        cur_test = getattr(smoketests, tname)(tname)
    except AttributeError:
        print("Exiting: %s is not a valid smoke test." % tname)
        sys.exit(1)
    except:
        print("Exiting: failed during intialization.")
        traceback.print_exc()
        sys.exit(1)

    cur_test.start_test()
    cur_test.finish_test(fname)
    if (not proceed and cur_test.result[0] == False or
    'KeyboardInterrupt' in cur_test.result[1]):
        print("Exiting: Failed Test %s" % tname)
        sys.exit(1)


def test_help():
    """
    Help text for argparse describing available sdk tests.
    """
    return """Specify an individual test for execution
    TESTS:
    Enable    - Verify ability to enable, check state and disable baxter
    Messages  - Verify messages being published and ability to subscribe
    Services  - Verify services available and ability to make calls as client
    Head      - Move the head pan and tilt, display image to screen
    MoveArms  - Move both arms through entire joint range
    Grippers  - Calibrate and move grippers using position and velocity control
    BlinkLEDs - Blink Navigator LEDs
    Cameras   - Verify camera publishing and visualization
    """


def get_version():
    """
    Get current software version number from param server.
    """
    try:
        version = rospy.get_param('/rethink/software_version').split('_')[0]
    except socket.error:
        print("Exiting: Could not communicate with ROS Master to determine " +
              "Software version")
        sys.exit(1)
    except:
        print("Exiting: Could not determine SW version from param " +
            "'/rethink/software_version'")
        sys.exit(1)
    return version


def ros_init():
    """
    Initialize rsdk_smoke_test ros node.
    """
    print("Initializing node 'rsdk_smoke_test'\n")
    rospy.init_node('rsdk_smoke_test', disable_signals=True)


def main():
    format = argparse.RawTextHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=format)
    parser.add_argument('-p', '--proceed', action='store_true',
        help="Continue testing after a failed test until all tests complete")
    parser.add_argument('-t', '--test', help=test_help())
    args = parser.parse_args(rospy.myargv()[1:])

    test_dict = {
        'version': None,
        'valid_tests': {
            '0.7.0': ['Enable', 'Messages', 'Services', 'Head', 'MoveArms',
                'Grippers', 'BlinkLEDs', 'Cameras'],
            }
        }

    test_dict['version'] = get_version()
    if not test_dict['version'] in test_dict['valid_tests'].keys():
        print("Exiting: No tests specified for your software version: %s" %
            (test_dict['version']))
        return 1

    try:
        raw_input("Press <Enter> to Begin Smoke Test\n")
    except Exception:
        print("\nExiting.")
        return 1

    hostname = re.split('http://|.local', rospy.get_master().getUri()[2])[1]
    cur_time = time.localtime()
    filename = ("%s-%s.%s.%s-rsdk-%s.smoketest" %
                (hostname, cur_time.tm_mon, cur_time.tm_mday,
                 cur_time.tm_year, test_dict['version'],)
                )
    if args.test == None:
        print 'Performing All Tests'
        ros_init()
        for t in test_dict['valid_tests'][test_dict['version']]:
            run_test(t, filename, args.proceed)
    elif args.test in test_dict['valid_tests'][test_dict['version']]:
        ros_init()
        run_test(args.test, filename, args.proceed)
    else:
        print("Exiting: Invalid test provided: %s for %s version software" %
              (args.test, test_dict['version']))
        parser.print_help()

    return 0

if __name__ == '__main__':
    sys.exit(main())
