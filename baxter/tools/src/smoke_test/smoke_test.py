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
import socket
import traceback

import roslib
roslib.load_manifest('tools')
import rospy

import smoketests

class TestDict():
    """Create structure to hold releases with assosciated tests. Load and run the tests.
    """
    def __init__(self, version):
        self._version = version
        self._valid_tests = {}
        self._valid_tests.update({'1.1.0': ['Enable', 'Messages', 'Services', 'Head', 'MoveArms', 'Grippers', 'BlinkLEDs', 'Cameras']})

    def validate(self):
        return self._version in self._valid_tests

    def run_test(self, tname, fname, proceed):
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
        if not proceed and cur_test._result[0][0] == 'F' or 'KeyboardInterrupt' in cur_test._result[1]:
           print("Exiting: Failed Test %s" % tname)
           sys.exit(1)

def test_help():
    return """Specify an individual test for execution
    TESTS:
    Enable     - Verify ability to enable, check state and disable baxter.
    Messages   - Verify messages being published and ability to subscribe.
    Services   - Verify services available and ability to make calls as client.
    Head       - Move the head pan and tilt, display image to screen
    MoveArms   - Move both arms through entire joint range.
    Grippers   - Calibrate and move grippers using position and velocity control.
    BlinkLEDs  - Blink Navigator LEDs.
    Cameras    - Verify camera publishing and visualization.
    """

def get_version():
    """Get current software version number from param server
    """
    try:
        v = rospy.get_param('/rethink/software_version').split('_')[0]
    except socket.error:
        print("Exiting: Could not communicate with ROS Master to determine SW version")
        sys.exit(1)
    except:
        print("Exiting: Could not determine SW version from param '/jrcp/version'")
        sys.exit(1)
    return v

def ros_init():
    print("Initializing node 'rsdk_smoke_test'\n")
    rospy.init_node('rsdk_smoke_test', disable_signals=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-p', '--proceed', action='store_true', help="Continue testing after a failed test until all tests complete")
    parser.add_argument('-t', '--test', help=test_help())
    args = parser.parse_args()

    version = get_version()
    test = TestDict(version)
    if not test.validate():
        print("Exiting: No tests specified for your software version: %s" % version)
        sys.exit(1)

    try:
        raw_input("Press <Enter> to Begin Smoke Test\n")
    except:
        print("\nExiting.")
        sys.exit(1)

    cur_time = time.localtime()
    filename = ("rsdk-smoke_%s_%s.%s.%s" % (version, cur_time.tm_mday, cur_time.tm_mon, cur_time.tm_year))
    if args.test == None:
        print 'Performing All Tests'
        ros_init()
        for t in test._valid_tests[version]:
            test.run_test(t, filename, args.proceed)
    elif args.test in test._valid_tests[version]:
        ros_init()
        test.run_test(args.test, filename, args.proceed)
    else:
        print("Exiting: Invalid test provided: %s" % args.test)
        parser.print_help()
