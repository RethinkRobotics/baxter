import roslib
roslib.load_manifest('joint_pose')
import rospy

from std_msgs.msg import (Empty, Float32,)
from sensor_msgs.msg import JointState
from baxter_msgs.msg import (GripperState, JointCommandMode, JointPositions,)

class BaxterController(object):
    """ Interface for a Baxter robot controller
    """

    def command(self, commands):
        """ generic command function
        Args:
            commands(dict{str:float}): a set of name/value pairs
                that forms a robot command
        """
        raise NotImplementedError()


class GripperBaxterController(BaxterController):
    """ Controls a gripper on a Baxter Robot
    """
    def __init__(self, arm):
        self.pubCalibrate = rospy.Publisher('/robot/limb/' + arm + '/accessory/gripper/command_calibrate', Empty)
        self.pubGoto = rospy.Publisher('/robot/limb/' + arm + '/accessory/gripper/command_goto', Float32)
        self.subState = rospy.Subscriber('/robot/limb/' + arm + 'accessory/gripper/state', GripperState, self._gripperState)
        self.arm = arm
        self.calibrated = False
        self.position = 100.0 #open
        self.force = 0.0

    def _gripperState(self, msg):
        self.position = msg.position
        self.force = msg.force
        self.calibrated = msg.calibrated

    def command(self, commands):
        if not self.calibrated:
            self.pubCalibrate.publish(Empty())
        else:
            self.calibrating = False
            if 'position' in commands:
                position = commands['position']
                self.pubGoto.publish(Float32(position))
                self.position = position
            else:
                print("gripper %s does not know how to handle command(s) '%s'. Ignored" % (self.arm, str(commands.keys())))


class JointPositionBaxterController(BaxterController):
    """ Joint Position Controller for a Rethink Robotics Baxter RSDK Robot
    Controls a baxter robot by setting joint angles by combining joint angles
    read from the robot with user commands
    TODO: would make sense to split this out per arm
    """


    def __init__(self):
        """ Services subscribers and publishers for communication with Baxter
        Specifically for Joint Position Control
        Args:
            outputFileName(str):optional, when provided can be used to record joint
            positions
        """
        super(JointPositionBaxterController, self).__init__()
        self.left_position = {}
        self.right_position = {}
        self.pubLeftMode = rospy.Publisher('/robot/limb/left/joint_command_mode', JointCommandMode)
        self.pubRightMode = rospy.Publisher('/robot/limb/right/joint_command_mode', JointCommandMode)
        self.pubLeft = rospy.Publisher('/robot/limb/left/command_joint_angles', JointPositions)
        self.pubRight = rospy.Publisher('/robot/limb/right/command_joint_angles', JointPositions)
        self.subLeft = rospy.Subscriber('/robot/limb/left/joint_states', JointState, self.leftJointState)
        self.subRight = rospy.Subscriber('/robot/limb/right/joint_states', JointState, self.rightJointState)
        self.gripperLeft = GripperBaxterController("left")
        self.gripperRight = GripperBaxterController("right")
        self.newFile = True
        self.startTime = rospy.Time.now()

    def setPositionMode(self):
        """  Set Baxter's joint command mode to Position Control
        publishes the desire to put the robot in position control mode
        """
        msg = JointCommandMode()
        msg.mode = JointCommandMode.POSITION
        self.pubLeftMode.publish(msg)
        self.pubRightMode.publish(msg)

    def leftJointState(self, data):
        """ callback function for the ROS subscriber of the left joint angles
        Args:
            data(sensor_msgs.msg.JointState): the ROS message containing the joint state
        """
        for i in range(len(data.name)):
            self.left_position[data.name[i]] = data.position[i]


    def rightJointState(self, data):
        """ callback function for the ROS subscriber of the right joint angles
        Args:
            data(sensor_msgs.msg.JointState): the ROS message containing the joint state
        """
        for i in range(len(data.name)):
            self.right_position[data.name[i]] = data.position[i]


    def command(self, joint_positions, pos_is_delta=True):
        """ Joint command function
        Publishes new joint angles based on the supplied deltas
        and the most recently received joint angle state.
        Args:
            joint_positions(dict{str:int}): a set of name/value pairs
                each representing a joint name and a joint position
                A subset of joints can be provided, but each joint must
                have appeared in a JointState msg (even for absolute cmds)
        Kwargs:
            pos_is_delta(bool): when defaulted to True, interpret the positions
                in jointPositions as deltas from the current position.
                when set to False, use the positions as absolute
        """
        leftMsg = JointPositions()
        rightMsg = JointPositions()
        for full_joint_name, pos in joint_positions.items():
            if not pos is None:
                side, joint_name = full_joint_name.split('_')
                if side == 'left' and joint_name in self.left_position:
                    if pos_is_delta:
                        pos += self.left_position[joint_name]
                    leftMsg.names.append(joint_name)
                    leftMsg.angles.append(pos)
                elif side == 'right' and joint_name in self.right_position:
                    if pos_is_delta:
                        pos += self.right_position[joint_name]
                    rightMsg.names.append(joint_name)
                    rightMsg.angles.append(pos)
                elif joint_name == 'gripper':
                    if side == 'left':
                        self.gripperLeft.command({'position':pos})
                    elif side == 'right':
                        self.gripperRight.command({'position':pos})

        self.setPositionMode()
        self.pubLeft.publish(leftMsg)
        self.pubRight.publish(rightMsg)




