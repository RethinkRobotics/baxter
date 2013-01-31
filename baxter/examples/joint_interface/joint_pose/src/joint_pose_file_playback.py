#!/usr/bin/env python
#import os.path
#import sys
import argparse

import roslib
roslib.load_manifest('joint_pose')
import rospy
import enable_robot

from Mapper import Mapper
from JointPositionBaxterController import JointPositionBaxterController

class FileMapper(Mapper):
  """ CSV File mapper
  Maps input read from a csv file to robot control
  """

  def __init__(self, controller, filename, loops):
    """ Maps csv file input to robot control
    Args:
      controller(BaxterController): a type of Baxter robot controller
      filename(str): path to csv file to read from
      loops(int): number of times to play back the file
    """
    super(FileMapper, self).__init__(controller)
    self.filename = filename
    self.startTime = rospy.Time.now()
    self.loops = loops

  def timeStamp(self):
    diff = rospy.Time.now() - self.startTime
    return diff.to_sec()

  def loop(self):
    self.done = False
    if self.loops == 0:
      while(not self.done):
        self.play_file()
    elif self.loops > 0:
      for i in range(self.loops):
        self.play_file()
    else:
      self.play_file()

  def play_file(self):
    """ Loops through csv file
    Does not loop indefinitely, but only until the file is read
    and processed. Reads each line, split up in columns and
    formats each line into a controller command in the form of
    name/value pairs. Names come from the column headers
    first column is the time stamp
    """
    rate = rospy.Rate(1000)
    print("playing back %s" % (self.filename))
    with open(self.filename, 'r') as f:
      lines = f.readlines()
    keys = lines[0].rstrip().split(',')
    for values in lines[1:]:
      print(values)
      def convert(x):
        try:
          return float(x)
        except ValueError:
          return None
      #convert the line of strings to a float or None
      values = [convert(x) for x in values.rstrip().split(',')]
      #zip the values with the joint names
      combined = zip(keys[1:], values[1:])
      #take out any tuples that have a none value
      cleaned = [x for x in combined if x[1] is not None]
      #convert it to a dictionary with only valid commands
      cmd = dict(cleaned)
      #command this set of commands until the next frame
      while (self.timeStamp() < values[0] and not self.done):
        self.controller.command(cmd, False)
        rate.sleep()


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("file", help="input file")
  parser.add_argument("-l", "--loops", help="number of times to loop the input file. 0=infinite.")
  args = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node("rethink_rsdk_joint_pose_file_playback")
  print("Getting robot state... ")
  rs = enable_robot.RobotState()
  print("Enabling robot... ")
  rs.enable()

  mapper = FileMapper(JointPositionBaxterController(), args.file, args.loops)

  mapper.loop()
  rs.disable()
