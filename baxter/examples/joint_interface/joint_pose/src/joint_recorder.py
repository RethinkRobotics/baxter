#!/usr/bin/env python

  def timeStamp(self):
    diff = rospy.Time.now() - self.startTime
    return diff.to_sec()

  def record(self):
    """ Records the current joint positions to a csv file
    if outputFilename was provided at construction
    this function will record the latest set of joint angles
    in a csv format.
    This function does not test to see if a file exists and
    will overwrite existing files.
    """
    if self.outputFilename:
      if self.newFile:
        with open(self.outputFilename, 'w') as f:
          f.write('time,')
          f.write(','.join(self.leftPosition.keys()) + ',')
          f.write('left_gripper,')
          f.write(','.join(self.rightPosition.keys()) + ',')
          f.write('right_gripper\n')
        self.newFile = False

      with open(self.outputFilename, 'a') as f:
        f.write("%f," % (self.timeStamp(),))
        f.write(','.join([str(x) for x in self.leftPosition.values()]) + ',')
        f.write(str(self.gripperLeft.position) + ',')
        f.write(','.join([str(x) for x in self.rightPosition.values()]) + ',')
        f.write(str(self.gripperRight.position) + '\n')

if __name__ == '__main__':
  parser.add_option("-o", "--output",   dest="outputFilename", help="filename for output")
  parser.add_option("-r", "--record-rate",   dest="recordRate", help="rate at which to record automatically")

  self.subLeft = rospy.Subscriber('/robot/limb/left/joint_states', JointState, self.leftJointState)
  self.subRight = rospy.Subscriber('/robot/limb/right/joint_states', JointState, self.rightJointState)
