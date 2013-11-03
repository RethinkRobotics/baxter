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


import argparse
import os
import sys

import rospy

import baxter_interface

from baxter_maintenance_msgs.msg import (
    TareEnable,
)


class Tare(baxter_interface.RobustController):
    def __init__(self, limb):
        """
        Wrapper to run the Tare RobustController.

        @param limb - Limb to run tare on [left/right]
        """
        enable_msg = TareEnable(isEnabled=True, uid='sdk')
        enable_msg.data.tuneGravitySpring = True

        disable_msg = TareEnable(isEnabled=False, uid='sdk')

        # Initialize RobustController, use 5 minute timeout for the Tare
        # process.
        super(Tare, self).__init__(
            'robustcontroller/%s/Tare' % (limb,),
            enable_msg,
            disable_msg,
            5 * 60)


def main():
    parser = argparse.ArgumentParser()
    required = parser.add_argument_group('required arguments')
    required.add_argument('-l', '--limb', required=True,
                        choices=['left', 'right'],
                        help='Tare the specified limb')
    args = parser.parse_args(rospy.myargv()[1:])
    limb = args.limb

    rospy.init_node('tare_sdk', anonymous=True)
    rs = baxter_interface.RobotEnable()
    rs.enable()
    tt = Tare(limb)
    rospy.loginfo("Running tare on %s limb" % (limb,))

    error = None
    try:
        tt.run()
    except Exception, e:
        error = e.strerror
    finally:
        try:
            rs.disable()
        except Exception:
            pass

    if error == None:
        rospy.loginfo("Tare finished")
    else:
        rospy.logerr("Tare failed: %s" % (error,))

    return 0 if error == None else 1

if __name__ == '__main__':
    sys.exit(main())
