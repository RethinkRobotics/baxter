import roslib
roslib.load_manifest('joint_position')
import rospy

import baxter_interface
import iodevices


class JointRecorder(object):

    def __init__(self, filename, rate, connection_timeout=2.0):
        """ records joint data to a file at a specified rate

        """
        self._filename = filename
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False

        self._limb_left = baxter_interface.Limb("left")
        self._limb_right = baxter_interface.Limb("right")
        self._gripper_left = baxter_interface.Gripper("left")
        self._gripper_right = baxter_interface.Gripper("right")

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done
        """
        if rospy.is_shutdown():
            self.stop()
        elif iodevices.getch():
            self.stop()
        return self._done

    def record(self):
        """ Records the current joint positions to a csv file
        if outputFilename was provided at construction
        this function will record the latest set of joint angles
        in a csv format.
        This function does not test to see if a file exists and
        will overwrite existing files.
        """
        if self._filename:
            joints_left = self._limb_left.joints()
            joints_right = self._limb_right.joints()
            with open(self._filename, 'w') as f:
                f.write('time,')
                f.write(','.join(["left_" + j for j in joints_left]) + ',')
                f.write('left_gripper,')
                f.write(','.join(["right_" + j for j in joints_right]) + ',')
                f.write('right_gripper\n')

                while not self.done():
                    angles_left = [self._limb_left.joint_angle(j) for j in joints_left]
                    angles_right = [self._limb_right.joint_angle(j) for j in joints_right]

                    f.write("%f," % (self._time_stamp(),))

                    f.write(','.join([str(x) for x in angles_left]) + ',')
                    f.write(str(self._gripper_left.position()) + ',')

                    f.write(','.join([str(x) for x in angles_right]) + ',')
                    f.write(str(self._gripper_right.position()) + '\n')

                    self._rate.sleep()

