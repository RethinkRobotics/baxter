#!/usr/bin/env python
import signal
import argparse

from GripperBaxterController import GripperBaxterController
from sensor_msgs.msg import JointState

import roslib
roslib.load_manifest('joint_pose')
import rospy

class JointRecorder(object):

  def __init__(self, filename, rate, delta):
    self._filename = filename
    self._rate = rospy.Rate(rate)
    self._delta = delta
    self._startTime = rospy.Time.now()
    self._leftPosition = {}
    self._rightPosition = {}
    self._leftPositionFiltered = {}
    self._rightPositionFiltered = {}
    self._done = False

    #TODO: factor out arm interfaces and reuse in JointPositionBaxterController
    self._subLeft = rospy.Subscriber('/robot/limb/left/joint_states', JointState, self.leftJointState)
    self._subRight = rospy.Subscriber('/robot/limb/right/joint_states', JointState, self.rightJointState)
    self._gripperLeft = GripperBaxterController('left')
    self._gripperRight = GripperBaxterController('right')

    signal.signal(signal.SIGINT, self.handleCtrlC)
    self.waitForData()

  def handleCtrlC(self, signum, frame):
    """ ctrl-c handler
    """
    self.stop()

  def stop(self):
    self._done = True

  def done(self):
    return self._done or rospy.is_shutdown()

  def leftJointState(self, data):
    """ callback function for the ROS subscriber of the left joint angles
    The previously recorded position is checked against the new data and the delta
    If the new position exceeds the delta from the previous position, it is updated
    If the position is not updated, the filtered copy is set to "None", to indicate
    that no command needs to be recorded
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      key = 'left_'+data.name[i]
      if key not in self._leftPosition or abs(self._leftPosition[key] - data.position[i]) > self._delta:
        self._leftPosition[key] = data.position[i]
        self._leftPositionFiltered[key] = data.position[i]
      else:
        self._leftPositionFiltered[key] = None

  def rightJointState(self, data):
    """ callback function for the ROS subscriber of the right joint angles
    The previously recorded position is checked against the new data and the delta
    If the new position exceeds the delta from the previous position, it is updated
    If the position is not updated, the filtered copy is set to "None", to indicate
    that no command needs to be recorded
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      key = 'right_'+data.name[i]
      if key not in self._rightPosition or abs(self._rightPosition[key] - data.position[i]) > self._delta:
        self._rightPosition[key] = data.position[i]
        self._rightPositionFiltered[key] = data.position[i]
      else:
        self._rightPositionFiltered[key] = None

  def timeStamp(self):
    diff = rospy.Time.now() - self._startTime
    return diff.to_sec()

  def waitForData(self):
    rate = rospy.Rate(100)
    while not self.done():
      if len(self._leftPosition.keys()) and len(self._rightPosition.keys()):
        return True
      rate.sleep()
    return False

  def record(self):
    """ Records the current joint positions to a csv file
    if outputFilename was provided at construction
    this function will record the latest set of joint angles
    in a csv format.
    This function does not test to see if a file exists and
    will overwrite existing files.
    """
    lGripPos = self._gripperLeft.position
    rGripPos = self._gripperRight.position
    if self._filename:
      with open(self._filename, 'w') as f:
        f.write('time,')
        f.write(','.join(self._leftPosition.keys()) + ',')
        f.write('left_gripper,')
        f.write(','.join(self._rightPosition.keys()) + ',')
        f.write('right_gripper\n')

        while not self.done():
          lVals = self._leftPositionFiltered.values()
          rVals = self._rightPositionFiltered.values()
          lGripChange = abs(lGripPos - self._gripperLeft.position) > self._delta
          rGripChange = abs(rGripPos - self._gripperRight.position) > self._delta
          if any(lVals) or any(rVals) or lGripChange or rGripChange:
            f.write("%f," % (self.timeStamp(),))
            f.write(','.join([str(x) if x else "" for x in lVals]) + ',')
            f.write(str(self._gripperLeft.position) + ',' if lGripChange else ',')
            f.write(','.join([str(x) if x else "" for x in rVals]) + ',')
            f.write(str(self._gripperRight.position) + '\n' if rGripChange else '\n')
            lGripPos = self._gripperLeft.position
            rGripPos = self._gripperRight.position
          self._rate.sleep()

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("filename", help="the file name to record to")
  parser.add_argument("-r", "--record-rate", dest="recordRate", type=int, default=100, help="rate at which to record")
  parser.add_argument("-d", "--joint-delta", dest="jointDelta", type=float, default=0.1, help="joint delta to trigger recording a frame")
  args = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node("rethink_rsdk_joint_recorder")
  recorder = JointRecorder(args.filename, args.recordRate, args.jointDelta)
  print("recording...")
  recorder.record()
  print("done.")
