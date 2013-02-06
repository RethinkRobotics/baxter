import roslib
roslib.load_manifest('joint_position')
import rospy

from controllers import GripperBaxterController
from sensor_msgs.msg import JointState
import signal


class JointRecorder(object):

  def __init__(self, filename, rate):
    """ records joint data to a file at a specified rate

    """
    self._filename = filename
    self._rate = rospy.Rate(rate)
    self._startTime = rospy.Time.now()
    self._leftPosition = {}
    self._rightPosition = {}
    self._done = False

    #TODO: factor out arm interfaces and reuse in JointPositionBaxterController
    self._subLeft = rospy.Subscriber('/robot/limb/left/joint_states', JointState, self._left_joint_state)
    self._subRight = rospy.Subscriber('/robot/limb/right/joint_states', JointState, self._right_joint_state)
    self._gripperLeft = GripperBaxterController('left')
    self._gripperRight = GripperBaxterController('right')

    signal.signal(signal.SIGINT, self._handle_ctrl_c)
    self.wait_for_data()

  def _handle_ctrl_c(self, signum, frame):
    """ ctrl-c handler
    """
    self.stop()

  def stop(self):
    self._done = True

  def done(self):
    return self._done or rospy.is_shutdown()

  def _left_joint_state(self, data):
    """ callback function for the ROS subscriber of the left joint angles
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      key = 'left_'+data.name[i]
      self._leftPosition[key] = data.position[i]

  def _right_joint_state(self, data):
    """ callback function for the ROS subscriber of the right joint angles
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      key = 'right_'+data.name[i]
      self._rightPosition[key] = data.position[i]

  def _time_stamp(self):
    diff = rospy.Time.now() - self._startTime
    return diff.to_sec()

  def wait_for_data(self):
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
    if self._filename:
      with open(self._filename, 'w') as f:
        f.write('time,')
        f.write(','.join(self._leftPosition.keys()) + ',')
        f.write('left_gripper,')
        f.write(','.join(self._rightPosition.keys()) + ',')
        f.write('right_gripper\n')

        while not self.done():
          lVals = self._leftPosition.values()
          rVals = self._rightPosition.values()

          f.write("%f," % (self._time_stamp(),))

          f.write(','.join([str(x) for x in lVals]) + ',')
          f.write(str(self._gripperLeft.position) + ',')

          f.write(','.join([str(x) for x in rVals]) + ',')
          f.write(str(self._gripperRight.position) + '\n')

          self._rate.sleep()


class SparseRecorder(JointRecorder):

  def __init__(self, filename, rate, delta):
    """ records joint data to a file  t a base max rate
    Does not record joints that did not change beyond
    the specified delta

    """
    super(SparseRecorder, self).__init__(filename, rate)
    self._delta = delta
    self._leftPositionFiltered = {}
    self._rightPositionFiltered = {}

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

  def record(self):
    """ Records the current joint positions to a csv file
    if outputFilename was provided at construction
    this function will record the latest set of joint angles
    in a csv format.
    This function does not test to see if a file exists and
    will overwrite existing files.
    TODO: Fix. either here or in playback something goes wrong;
    it appears joints get mixed up.
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


