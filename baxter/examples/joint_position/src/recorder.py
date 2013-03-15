#!/usr/bin/env python
import argparse
import roslib
roslib.load_manifest('joint_position')
import rospy

from jointrecorder import JointRecorder

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("filename", help="the file name to record to")
  parser.add_argument("-r", "--record-rate", dest="recordRate", type=int, default=10, help="rate at which to record")
  args = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node("rethink_rsdk_joint_recorder")
  recorder = JointRecorder(args.filename, args.recordRate)
  print("Recording. Press any key to stop.")
  recorder.record()
  print("done.")
