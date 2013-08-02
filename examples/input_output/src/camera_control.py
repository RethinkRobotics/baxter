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
import baxter_msgs.srv
def usage():
    print """
%s [ARGUMENTS]

    -h, --help                  This screen
    -o, --open [CAMERA]         Open specified camera
    -c, --close [CAMERA]        Close specified camera
    -r, --resolution [X]x[Y]    Set camera resolution
    -l, --list                  List available cameras
    """ % (os.path.basename(sys.argv[0]),)

def list_cameras(*args, **kwds):
    ls = rospy.ServiceProxy('/cameras/list', baxter_msgs.srv.ListCameras)
    rospy.wait_for_service('/cameras/list', timeout = 10)
    resp = ls()
    if len(resp.cameras):
        print ('Cameras:')
        print ('\n'.join(resp.cameras))
    else:
        print ('No cameras found')

def open_camera(camera, res, *args, **kwds):
    cam = baxter_interface.CameraController(camera)
    cam.close()
    cam.resolution = res
    cam.open()

def close_camera(camera, *args, **kwds):
    cam = baxter_interface.CameraController(camera)
    cam.close()

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'ho:c:r:l',
            ['help', 'open=', 'close=', 'resolution=', 'list'])
    except getopt.GetoptError as err:
        print str(err)
        usage()
        sys.exit(2)

    action = None
    camera = None
    res = (1280, 800)

    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif o in ('-o', '--open'):
            camera = a
            action = open_camera
        elif o in ('-c', '--close'):
            camera = a
            action = close_camera
        elif o in ('-r', '--resolution'):
            res = a.split('x')
            if len(res) != 2:
                print ("Invalid resolution.")
                sys.exit(1)
            res = (int(res[0]), int(res[1]))
        elif o in ('-l', '--list'):
            action = list_cameras

    if action == None:
        print ("No action defined")
        usage()
        sys.exit(2)

    rospy.init_node('cameras_example', anonymous = True)
    action(camera, res)
    sys.exit(0)

if __name__ == '__main__':
    main()

