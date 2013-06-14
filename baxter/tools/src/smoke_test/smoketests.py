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
Baxter RSDK Smoke Tests
"""

import os
import traceback
import threading
import Queue
import dataflow

import roslib
roslib.load_manifest('tools')
import rospy
import std_msgs
from sensor_msgs.msg import (
    JointState,
    Image,
    )
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
    )
import cv
import cv_bridge

import baxter_interface
from baxter_msgs.msg import (
    AnalogIOStates,
    GripperState,
    GripperIdentity,
    )
from baxter_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    ListCameras,
    )

class SmokeTest(object):
    """smoke tests base class
    """
    def __init__(self, name):
        self._name = name
        self._rs = baxter_interface.RobotEnable()
        self.result = [False, '']

    def start_test(self):
        """Set up pub/sub/actions and run test.
        Results return in form [Bool Success,'Stacktrace if Failure'] 
        """
        raise NotImplementedError()

    def finish_test(self, filename):
        """Store results to output file.
        """

        with open(filename, "a") as testlog:
            testlog.write(
                "Test: %s ------ Result: %s\n" % 
                (self._name, "Success" if self.result[0] else "Failure",)
                )
            if self.result[0] == False:
                testlog.write("%s\n%s%s\n" % 
                    ('*' * 40, self.result[1], '*' * 40,)
                    )
        print("Results of %s saved to file: %s" % (self._name, filename,))
        print("------- Result: %s -------\n\n" % 
            ("Success" if self.result[0] else "Failure",)
            )

    def return_failure(self, trace):
        """Commonly used failure return
        """
        self.result[0] = False
        self.result[1] = trace
        try:
            self._rs.disable()
        except:
            pass

class Enable(SmokeTest):
    """Verify ability to enable, check state and disable baxter.
    """
    def __init__(self, name='Enable'):
        super(Enable, self).__init__(name)

    def start_test(self):
        """Runs Enable Smoke Test
        """
        try:
            print("Test: Enable - Zero G for 10 Seconds")
            self._rs.enable()
            # Allow User Zero G Testing
            rospy.sleep(10.0)
            print("Test: State")
            print self._rs.state()
            print("Test: Disable")
            self._rs.disable()
            self.result[0] = True
        except:
            self.return_failure(traceback.format_exc())

class Messages(SmokeTest):
    """Verify messages being published and ability to subscribe.
    """
    def __init__(self, name='Messages'):
        super(Messages, self).__init__(name)

    def start_test(self):
        """Runs Messages Smoke Test
        """
        try:
            print("Test: Subscribe to topic: /robot/joint_states")
            rospy.wait_for_message('/robot/joint_states', JointState, 5.0)
            print("Test: Subscribe to topic: /sdk/robot/analog_io_states")
            rospy.wait_for_message(
                '/sdk/robot/analog_io_states', 
                AnalogIOStates, 
                5.0
                )
            self.result[0] = True
        except:
            self.return_failure(traceback.format_exc())

class Services(SmokeTest):
    """Verify services available and ability to make calls as client.
    """
    def __init__(self, name='Services'):
        super(Services, self).__init__(name)

    def start_test(self):
        """Runs Services Smoke Test
        """
        try:
            srv = '/sdk/robot/limb/left/solve_ik_position'
            print("Test: Service availability: %s" % srv)
            rospy.wait_for_service(srv, 5.0)
            print("Test: IK service call Solve IK")
            iksvc = rospy.ServiceProxy(srv, SolvePositionIK)
            ikreq = SolvePositionIKRequest()
            pose = PoseStamped(
                header=std_msgs.msg.Header(
                stamp=rospy.Time.now(), frame_id='base'),
                pose=Pose(
                    position=Point(
                        x=0.657579481614,
                        y=0.851981417433,
                        z=0.0388352386502,
                        ),
                    orientation=Quaternion(
                        x= -0.366894936773,
                        y=0.885980397775,
                        z=0.108155782462,
                        w=0.262162481772,
                        ),
                     )
                 )
            ikreq.pose_stamp.append(pose)
            iksvc(ikreq)
            self.result[0] = True
        except:
            self.return_failure(traceback.format_exc())

class Head(SmokeTest):
    '''Move the head pan and tilt, display image to screen
    '''
    def __init__(self, name='Head'):
        super(Head, self).__init__(name)

    def start_test(self):
        """Runs MoveArms Smoke Test
        """
        try:
            print("Enabling robot...")
            self._rs.enable()
            print("Test: Moving Head to Neutral Location.")
            head = baxter_interface.Head()
            head.set_pan(0.0, 5.0)
            print("Test: Pan Head")
            head.set_pan(1.0, 5.0)
            head.set_pan(-1.0, 5.0)
            head.set_pan(0.0, 5.0)
            print("Test: Nod Head")
            for x in range(3):
                head.command_nod()
            print("Test: Display Image on Screen - 5 seconds.")
            image_path = os.path.dirname(os.path.abspath(__file__))
            img = cv.LoadImage(image_path + '/baxterworking.png')
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub = rospy.Publisher('/sdk/xdisplay', Image, latch=True)
            pub.publish(msg)
            rospy.sleep(5.0)
            img = cv.LoadImage(image_path + '/researchsdk.png')
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            pub.publish(msg)
            print("Disabling robot...")
            self._rs.disable()
            self.result[0] = True
        except:
            self.return_failure(traceback.format_exc())

class MoveArms(SmokeTest):
    """Move both arms to numerous joint positions.
    """
    def __init__(self, name='MoveArms'):
        super(MoveArms, self).__init__(name)

    def start_test(self):
        """Runs MoveArms Smoke Test
        """

        def move_thread(limb, angle, queue, timeout=15.0):
            """Threaded joint movement allowing for simultaneous joint moves
            """
            try:
                limb.move_to_joint_positions(angle, timeout)
                queue.put(None)
            except Exception, exception:
                queue.put(traceback.format_exc())
                queue.put(exception)

        try:
            print("Enabling robot...")
            self._rs.enable()
            print("Test: Create Limb Instances")
            right = baxter_interface.Limb('right')
            left = baxter_interface.Limb('left')
            left_queue = Queue.Queue()
            right_queue = Queue.Queue()
            # Max Joint Range (s0, s1, e0, e1, w0, w1, w2)
            #     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
            # Min Joint Range (e0, e1, s0, s1, w0, w1, w2) 
            #     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)
            joint_moves = (
                [ 0.0, -0.55, 0.0, 0.75, 0.0, 1.26,  0.0],
                [ 0.5,  -0.8, 2.8, 0.15, 0.0,  1.9,  2.8],
                [-0.1,  -0.8,-1.0, 2.5,  0.0, -1.4, -2.8],
                [ 0.0, -0.55, 0.0, 0.75, 0.0, 1.26,  0.0],
                )
            for move in joint_moves:
                print(
                    "Test: Moving to Joint Positions: " + \
                    ", ".join("%.2f" % x for x in move)
                    )
                left_thread = threading.Thread(
                    target=move_thread, 
                    args=(left, dict(zip(left.joint_names(), move)), left_queue)
                    )
                right_thread = threading.Thread(
                    target=move_thread, 
                    args=(right, dict(zip(right.joint_names(), move)), right_queue)
                    )
                left_thread.daemon = True
                right_thread.daemon = True
                left_thread.start()
                right_thread.start()
                dataflow.wait_for(
                    lambda: \
                    not (left_thread.is_alive() or right_thread.is_alive()),
                    timeout=20.0,
                    rate=10,
                    )
                left_thread.join()
                right_thread.join()
                result = left_queue.get()
                if not result == None:
                    raise left_queue.get()
                result = right_queue.get()
                if not result == None:
                    raise right_queue.get()
                rospy.sleep(1.0)
            print("Disabling robot...")
            self._rs.disable()
            self.result[0] = True
            rospy.sleep(5.0)
        except:
            self.return_failure(traceback.format_exc())

class Grippers(SmokeTest):
    """Calibrate and move grippers using torque, position, velocity control.
    """
    def __init__(self, name='Grippers'):
        super(Grippers, self).__init__(name)

    def start_test(self):
        """Runs Grippers Smoke Test
        """
        try:
            print("Enabling robot, Moving to Neutral Location...")
            self._rs.enable()
            right = baxter_interface.Limb('right')
            left = baxter_interface.Limb('left')
            right.move_to_neutral()
            left.move_to_neutral()
            rospy.sleep(2.0)
            print("Test: Verify Gripper Identity")
            gripper_id = rospy.wait_for_message(
                '/sdk/robot/limb/right/accessory/gripper/identity',
                GripperIdentity,
                5.0
                )
            if not 'electric gripper' in gripper_id.name:
                raise NameError("Grippers Test Requires Two Electric Grippers")
            gripper_id = rospy.wait_for_message(
                '/sdk/robot/limb/left/accessory/gripper/identity',
                GripperIdentity,
                5.0
                )
            if not 'electric gripper' in gripper_id.name:
                raise NameError("Grippers Test Requires Two Electric Grippers")
            print("Test: Subscribe to Gripper State.")
            rospy.wait_for_message(
                '/sdk/robot/limb/left/accessory/gripper/state',
                GripperState,
                5.0
                )
            rospy.wait_for_message(
                '/sdk/robot/limb/right/accessory/gripper/state',
                GripperState,
                5.0
                )
            print("Test: Reboot Grippers.")
            lg = baxter_interface.Gripper('left')
            rg = baxter_interface.Gripper('right')
            lg.reboot()
            rg.reboot()
            print("Test: Calibrating Grippers.")
            lg.calibrate()
            rg.calibrate()
            print("Test: Open Grippers")
            lg.open()
            rg.open()
            print("Test: Close Grippers")
            rospy.sleep(2.0)
            lg.close()
            rg.close()
            rospy.sleep(2.0)
            print("Test: Open Grippers")
            lg.open()
            rg.open()
            rospy.sleep(2.0)
            print("Test: Position Moves")
            lg.set_position(50.0)
            rg.set_position(50.0)
            rospy.sleep(1.0)
            lg.set_position(75.0)
            rg.set_position(75.0)
            rospy.sleep(1.0)
            lg.set_position(0.0)
            rg.set_position(0.0)
            rospy.sleep(1.0)
            lg.set_position(100.0)
            rg.set_position(100.0)
            rospy.sleep(1.0)
            print("Disabling robot...")
            self._rs.disable()
            self.result[0] = True
        except:
            self.return_failure(traceback.format_exc())

class BlinkLEDs(SmokeTest):
    """Blink Navigator LEDs.
    """
    def __init__(self, name='BlinkLEDs'):
        super(BlinkLEDs, self).__init__(name)

    def start_test(self):
        """Runs BlinkLEDs Smoke Test
        """

        def _blink(io):
            """Toggle itb lights
            """
            for i in range(9):
                io.set_output(i % 2)
                rospy.sleep(0.1)

        try:
            itb_names = (
                'left_itb_light_outer',
                'left_itb_light_inner',
                'torso_left_itb_light_outer',
                'torso_left_itb_light_inner',
                'right_itb_light_outer',
                'right_itb_light_inner',
                'torso_right_itb_light_outer',
                'torso_right_itb_light_inner',
                )

            for itb in itb_names:
                print("Test: Blink %s" % itb)
                io = baxter_interface.DigitalIO(itb)
                _blink(io)
            self.result[0] = True
        except:
            self.return_failure(traceback.format_exc())

class Cameras(SmokeTest):
    """Verify camera publishing and visualization.
    """
    def __init__(self, name='Cameras'):
        super(Cameras, self).__init__(name)

    def start_test(self):
        """Runs Cameras Smoke Test
        """

        xpub_img = rospy.Publisher(
            '/sdk/xdisplay',
            Image
        )

        def _display(camera, name):
            """Open camera and display to screen for 10 seconds
            """
            camera.close()
            camera.resolution = (960, 600,)
            print("Test: Opening %s" % name)
            camera.open()
            print("Test: Display %s to Screen - 10 Seconds" % name)
            camera_sub = rospy.Subscriber(
                '/cameras/' + name + "/image",
                Image,
                _repub_cb
                )
            rospy.sleep(10.0)
            camera.close()
            _reset_screen()

        def _repub_cb(msg):
            """Camera image republish callback
            """
            xpub_img.publish(msg)

        def _reset_screen():
            """Reset the screen to research sdk image
            """
            image_path = os.path.join(
                os.path.dirname(
                    os.path.abspath(__file__)
                    ),
                os.path.basename(
                    'researchsdk.png'
                    ),
                )
            img = cv.LoadImage(image_path)
            msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
            xpub_img.publish(msg)

        def _reset_defaults():
            """Turn on the left/right_hand_cameras with default settings
            """
            print("Restarting the Default Cameras...")
            for i in range(1, 3):
                camera = baxter_interface.CameraController(camera_names[i])
                camera.resolution = (320, 200,)
                camera.fps = 25
                camera.open()

        try:
            print("Enabling robot...")
            self._rs.enable()
            print("Test: Verify Left_Hand, Right_Hand, and Head " \
                "Cameras Present")
            camera_names = (
                'head_camera',
                'left_hand_camera',
                'right_hand_camera',
                )
            srv = "/cameras/list"
            rospy.wait_for_service(srv, 5.0)
            camera_list_srv = rospy.ServiceProxy(srv, ListCameras)
            camera_list = camera_list_srv()
            for camera_name in camera_names:
                if not camera_name in camera_list.cameras:
                    raise NameError("Could not find camera - %s" % camera_name)
                else:
                    camera = baxter_interface.CameraController(camera_name)
                    camera.close()
            for camera_name in camera_names:
                camera = baxter_interface.CameraController(camera_name)
                _display(camera, camera_name)
            _reset_defaults()
            print("Disabling robot...")
            self._rs.disable()
            self.result[0] = True
        except:
            self.return_failure(traceback.format_exc())
