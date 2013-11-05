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

import errno

import rospy

from baxter_core_msgs.msg import (
    CameraControl,
    CameraSettings,
)
from baxter_core_msgs.srv import (
    CloseCamera,
    ListCameras,
    OpenCamera,
)


class CameraController(object):
    """
    Interface class for controlling camera settings on the Baxter robot.
    """

    # Valid resolutions
    MODES = [
             (1280, 800),
             (960, 600),
             (640, 400),
             (480, 300),
             (384, 240),
             (320, 200),
             ]

    # Used to represent when the camera is using automatic controls.
    # Valid for exposure, gain and white balance.
    CONTROL_AUTO = -1

    def __init__(self, name):
        """
        @param name -   camera identifier.  You can get a list of valid
                        identifiers by calling the ROS service /cameras/list.

                        Expected names are right_hand_camera, left_hand_camera
                        and head_camera.  However if the cameras are not
                        identified via the parameter server, they are simply
                        indexed starting at 0.
        """
        self._id = name

        list_svc = rospy.ServiceProxy('/cameras/list', ListCameras)
        rospy.wait_for_service('/cameras/list', timeout=10)
        if not self._id in list_svc().cameras:
            raise AttributeError("Invalid camera name '%s'" % (self._id,))

        self._open_svc = rospy.ServiceProxy('/cameras/open', OpenCamera)
        self._close_svc = rospy.ServiceProxy('/cameras/close', CloseCamera)

        self._settings = CameraSettings()
        self._settings.width = 320
        self._settings.height = 200
        self._settings.fps = 20
        self._open = False

    def _reload(self):
        if self._open:
            self.close()
            self.open()

    def _get_value(self, control, default):
        lookup = [c.value for c in self._settings.controls if c.id == control]
        try:
            return lookup[0]
        except IndexError:
            return default

    def _set_control_value(self, control, value):
        lookup = [c for c in self._settings.controls if c.id == control]
        try:
            lookup[0].value = value
        except IndexError:
            self._settings.controls.append(CameraControl(control, value))

    @property
    def resolution(self):
        """
        Camera resolution as a tuple.  (width, height).  Valid resolutions are
        listed as tuples in CameraController.MODES
        """
        return (self._settings.width, self._settings.height)

    @resolution.setter
    def resolution(self, res):
        res = tuple(res)
        if len(res) != 2:
            raise AttributeError("Invalid resolution specified")

        if not res in self.MODES:
            raise ValueError("Invalid camera mode %dx%d" % (res[0], res[1]))

        self._settings.width = res[0]
        self._settings.height = res[1]
        self._reload()

    @property
    def fps(self):
        """
        Camera frames per second
        """
        return self._settings.fps

    @fps.setter
    def fps(self, fps):
        self._settings.fps = fps
        self._reload()

    @property
    def exposure(self):
        """
        Camera exposure.  If autoexposure is on, returns
        CameraController.CONTROL_AUTO
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_EXPOSURE,
                               self.CONTROL_AUTO)

    @exposure.setter
    def exposure(self, exposure):
        """
        Camera Exposure.  Valid range is 0-100 or CameraController.CONTROL_AUTO
        """
        exposure = int(exposure)
        if (exposure < 0 or exposure > 100) and exposure != self.CONTROL_AUTO:
            raise ValueError("Invalid exposure value")

        self._set_control_value(CameraControl.CAMERA_CONTROL_EXPOSURE,
                                exposure)
        self._reload()

    @property
    def gain(self):
        """
        Camera gain.  If autogain is on, returns CameraController.CONTROL_AUTO
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_GAIN,
                               self.CONTROL_AUTO)

    @gain.setter
    def gain(self, gain):
        """
        Camera gain.  Range is 0-79 or CameraController.CONTROL_AUTO
        """
        gain = int(gain)
        if (gain < 0 or gain > 79) and gain != self.CONTROL_AUTO:
            raise ValueError("Invalid gain value")

        self._set_control_value(CameraControl.CAMERA_CONTROL_GAIN, gain)
        self._reload()

    @property
    def white_balance_red(self):
        """
        White balance red.  If autocontrol is on, returns
        CameraController.CONTROL_AUTO
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R,
                               self.CONTROL_AUTO)

    @white_balance_red.setter
    def white_balance_red(self, value):
        """
        White balance red.  Range is 0-4095 or CameraController.CONTROL_AUTO
        """
        value = int(value)
        if (value < 0 or value > 4095) and value != self.CONTROL_AUTO:
            raise ValueError("Invalid white balance value")

        self._set_control_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R,
                                value)
        self._reload()

    @property
    def white_balance_green(self):
        """
        White balance green.  If autocontrol is on, returns
        CameraController.CONTROL_AUTO
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G,
                               self.CONTROL_AUTO)

    @white_balance_green.setter
    def white_balance_green(self, value):
        """
        White balance green.  Range is 0-4095 or CameraController.CONTROL_AUTO
        """
        value = int(value)
        if (value < 0 or value > 4095) and value != self.CONTROL_AUTO:
            raise ValueError("Invalid white balance value")

        self._set_control_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G,
                                value)
        self._reload()

    @property
    def white_balance_blue(self):
        """
        White balance blue.  If autocontrol is on, returns
        CameraController.CONTROL_AUTO
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B,
                               self.CONTROL_AUTO)

    @white_balance_blue.setter
    def white_balance_blue(self, value):
        """
        White balance blue.  Range is 0-4095 or CameraController.CONTROL_AUTO
        """
        value = int(value)
        if (value < 0 or value > 4095) and value != self.CONTROL_AUTO:
            raise ValueError("Invalid white balance value")

        self._set_control_value(CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B,
                                value)
        self._reload()

    @property
    def window(self):
        """
        Camera windowing, returns a tuple, (x, y)
        """
        x = self._get_value(CameraControl.CAMERA_CONTROL_WINDOW_X,
                            self.CONTROL_AUTO)
        if (x == self.CONTROL_AUTO):
            return (tuple(map(lambda x: x / 2, self.resolution)) if
            self.half_resolution else
            self.resolution)
        else:
            return (x, self._get_value(CameraControl.CAMERA_CONTROL_WINDOW_Y,
                                       self.CONTROL_AUTO))

    @window.setter
    def window(self, win):
        """
        Set camera window.  The max size is a function of the current camera
        resolution and if half_resolution is enabled or not.
        """
        x, y = tuple(win)
        cur_x, cur_y = self.resolution
        limit_x = 1280 - cur_x
        limit_y = 800 - cur_y

        if self.half_resolution:
            limit_x /= 2
            limit_y /= 2

        if x < 0 or x > limit_x:
            raise ValueError("Max X window is %d" % (limit_x,))

        if y < 0 or y > limit_y:
            raise ValueError("Max Y window is %d" % (limit_y,))

        self._set_control_value(CameraControl.CAMERA_CONTROL_WINDOW_X, x)
        self._set_control_value(CameraControl.CAMERA_CONTROL_WINDOW_Y, y)
        self._reload()

    @property
    def flip(self):
        """
        Camera flip. Returns True if flip is enabled on the camera.
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_FLIP, False)

    @flip.setter
    def flip(self, value):
        self._set_control_value(CameraControl.CAMERA_CONTROL_FLIP,
                                int(value != 0))
        self._reload()

    @property
    def mirror(self):
        """
        Camera mirror. Returns True if mirror is enabled on the camera.
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_MIRROR, False)

    @mirror.setter
    def mirror(self, value):
        self._set_control_value(CameraControl.CAMERA_CONTROL_MIRROR,
                                int(value != 0))
        self._reload()

    @property
    def half_resolution(self):
        """
        Return True if binning/half resolution is enabled on the camera.
        """
        return self._get_value(CameraControl.CAMERA_CONTROL_RESOLUTION_HALF,
                               False)

    @half_resolution.setter
    def half_resolution(self, value):
        self._set_control_value(CameraControl.CAMERA_CONTROL_RESOLUTION_HALF,
                                int(value != 0))
        self._reload()

    def open(self):
        """
        Open the camera currently under control.
        """
        if self._id == 'head_camera':
            self._set_control_value(CameraControl.CAMERA_CONTROL_FLIP, True)
            self._set_control_value(CameraControl.CAMERA_CONTROL_MIRROR, True)
        ret = self._open_svc(self._id, self._settings)
        if ret.err != 0:
            raise OSError(ret.err, "Failed to open camera")
        self._open = True

    def close(self):
        """
        Close, if necessary the camera.
        """
        ret = self._close_svc(self._id)
        if ret.err != 0 and ret.err != errno.EINVAL:
            raise OSError(ret.err, "Failed to close camera")
        self._open = False
