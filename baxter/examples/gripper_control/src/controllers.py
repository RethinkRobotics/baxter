
import roslib
roslib.load_manifest('gripper_control')
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from baxter_msgs.msg import GripperIdentity
from baxter_msgs.msg import GripperProperties
from baxter_msgs.msg import GripperState
from baxter_msgs.msg import GripperCommand

class GripperController():
  """ Controls a gripper on a Baxter Robot
  """
  def __init__(self, arm):
    self.arm = arm
    self.gripper_name = self.arm + " gripper"
    self.base_path = '/robot/limb/' + arm + '/accessory/gripper/'

    self.pub_enable = rospy.Publisher(self.base_path + 'set_enabled', Bool)
    self.pub_reset = rospy.Publisher(self.base_path + 'command_reset', Bool)
    self.pub_calibrate = rospy.Publisher(self.base_path + 'command_calibrate', Empty)
    self.pub_actuate = rospy.Publisher(self.base_path + 'command_set', GripperCommand)

    self.sub_identity = rospy.Subscriber(self.base_path + 'identity', GripperIdentity, self.on_gripper_identity)
    self.sub_properties = rospy.Subscriber(self.base_path + 'properties', GripperProperties, self.on_gripper_properties)
    self.sub_state = rospy.Subscriber(self.base_path + 'state', GripperState, self.on_gripper_state)

    self.last_command = ""
    self.last_args = {}
    self.command_changed = False

    self.command_msg = GripperCommand(0.0, 30.0, 100.0, 80.0, 3.0)
    self.identity_msg =  GripperIdentity()
    self.properties_msg =  GripperProperties()
    self.state_msg = GripperState()

    self.identity_changed = True
    self.properties_changed = False
    self.state_changed = False

    self.identity_report = ""
    self.properties_report = ""
    self.state_report = ""
    self._format_identity_report(self.identity_msg)

    self.status_val = {
      GripperState.STATE_FALSE: "False",
      GripperState.STATE_TRUE:  "True",
      GripperState.STATE_UNKNOWN: "Unknown"
      }

  def enable(self, **args):
    enable = True
    if 'enable' in args:
      enable = args['enable']

    if self.command_changed:
      print "Enabling" if enable else "Disabling", self.gripper_name
    self.pub_enable.publish(Bool(enable))

  def reset(self, **args):
    reboot = True
    if 'reboot' in args:
      reboot = args['reboot']

    if self.command_changed:
      print "Rebooting" if reboot else "Resetting", self.gripper_name
    self.pub_reset.publish(Bool(reboot))

  def calibrate(self, **args):
    if self.command_changed:
      if self.state_msg.calibrated:
        print "%s already calibrated!" % (self.gripper_name)
      else:
        print "Calibrating", self.gripper_name
    self.pub_calibrate.publish(Empty())

  def _actuate_arg(self, key, current_val, args):
    value = 1.0
    if 'value' in args:
      value = args['value']
    if key in args:
      return args[key] * value
    elif key + '_inc' in args:
      return current_val + (args[key + '_inc'] * value)
    elif key + '_dec' in args:
      return current_val - (args[key + '_dec'] * value)
    else:
      return current_val

  def _keep_gripper_arg_in_range(self, arg, min_range=0.0, max_range=100.0):
    if arg < min_range:
      return  min_range
    if arg > max_range:
      return max_range
    return arg

  def actuate(self, **args):
    self.command_msg.position = self._keep_gripper_arg_in_range(
      self._actuate_arg('position', self.state_msg.position, args))
    self.command_msg.force    = self._keep_gripper_arg_in_range(
      self._actuate_arg('moving_force', self.command_msg.force, args))
    self.command_msg.velocity = self._keep_gripper_arg_in_range(
      self._actuate_arg('velocity', self.command_msg.velocity, args))
    self.command_msg.holding  = self._keep_gripper_arg_in_range(
      self._actuate_arg('holding_force', self.command_msg.holding, args))
    self.command_msg.deadZone = self._keep_gripper_arg_in_range(
      self._actuate_arg('dead_zone', self.command_msg.deadZone, args))

    if self.command_changed:
      print("Actuate %s: pos=%0.2f%%, mov=%0.2f%%, vel=%0.2f%%, hld=%0.2f%%, dzn=%0.2f%%" % (
          self.gripper_name, 
          self.command_msg.position, 
          self.command_msg.force, 
          self.command_msg.velocity, 
          self.command_msg.holding, 
          self.command_msg.deadZone))

    self.pub_actuate.publish(self.command_msg)


  def handle_command(self, command, **args):
    """ Handle a gripper command.
        command(string): command
        args: { 'arg': value, }
    """
    options = {
      'enable': self.enable,
      'reset': self.reset,
      'calibrate': self.calibrate,
      'actuate': self.actuate
      }

    try:
      if command in options:
        self.command_changed = (self.last_command != command) or (self.last_args != args)
        options[command](**args)
        self.last_command = command
        self.last_args = args
      else:
        print "Unrecognized gripper command %s! args = %s" % (command, args)
    except Exception as ex:
      print "Exception commanding the gripper: ", ex
      print "Gripper command '%s' failed! args = %s" % (command, args)

  def _format_identity_report(self, identity_msg):
    self.identity_report = "%s: %s; type %d rev. %d" % (self.gripper_name,
                                                        identity_msg.name,
                                                        identity_msg.type,
                                                        identity_msg.hardware_id
                                                        )

  def on_gripper_identity(self, identity_msg):
    """ callback function for the ROS subscriber of the gripper identity
    Args:
      identity(baxter_msgs.msg.GripperIdentity): the ROS message containing the
      gripper identity
    """
    self.identity_changed = self.identity_changed or (
      self.identity_msg.type    != identity_msg.type or
      self.identity_msg.hardware_id != identity_msg.hardware_id
      )

    if self.identity_changed:
      self._format_identity_report(identity_msg)

    self.identity_msg = identity_msg

  def _format_properties_report(self, properties_msg):
    self.properties_report = self.gripper_name
    if self.properties_msg.hasForce    != properties_msg.hasForce:
      self.properties_report += "\n  hasForce: %s" % (self.status_val[properties_msg.hasForce],)
      if self.properties_msg.hasPosition    != properties_msg.hasPosition:
        self.properties_report += "\n  hasPosition: %s" % (self.status_val[properties_msg.hasPosition],)

  def on_gripper_properties(self, properties_msg):
    """ callback function for the ROS subscriber of the gripper properties
    Args:
      properties(baxter_msgs.msg.GripperProperties): the ROS message containing the
      gripper properties
    """
    self.properties_changed = self.properties_changed or (
      self.properties_msg.hasForce    != properties_msg.hasForce or
      self.properties_msg.hasPosition != properties_msg.hasPosition
      )

    if self.properties_changed:
      self._format_properties_report(properties_msg)

    self.properties_msg = properties_msg

  def _position_changed(self, old_pos, new_pos):
    return abs(new_pos - old_pos) > 0.5

  def _force_changed(self, old_force, new_force):
    return abs(new_force - old_force) > 0.5

  def _format_state_report(self, state_msg):
    self.state_report = self.gripper_name
    if self.state_msg.enabled    != state_msg.enabled:
      self.state_report += "\n  enabled: %s" % (self.status_val[state_msg.enabled],)
    if self.state_msg.calibrated    != state_msg.calibrated:
      self.state_report += "\n  calibrated: %s" % (self.status_val[state_msg.calibrated],)
    if self.state_msg.ready    != state_msg.ready:
      self.state_report += "\n  ready: %s" % (self.status_val[state_msg.ready],)
    if self.state_msg.moving    != state_msg.moving:
      self.state_report += "\n  moving: %s" % (self.status_val[state_msg.moving],)
    if self.state_msg.gripping    != state_msg.gripping:
      self.state_report += "\n  gripping: %s" % (self.status_val[state_msg.gripping],)
    if self.state_msg.missed    != state_msg.missed:
      self.state_report += "\n  missed: %s" % (self.status_val[state_msg.missed],)
    if self.state_msg.error    != state_msg.error:
      self.state_report += "\n  error: %s" % (self.status_val[state_msg.error],)
    if self.state_msg.position    != state_msg.position:
      self.state_report += "\n  position: %0.2f" % (state_msg.position,)
    if self.state_msg.force    != state_msg.force:
      self.state_report += "\n  force: %0.2f" % (state_msg.force,)

  def on_gripper_state(self, state_msg):
    """ callback function for the ROS subscriber of the gripper state
    Args:
      state(baxter_msgs.msg.GripperState): the ROS message containing the
      gripper state
    """
    self.state_changed = self.state_changed or (
      self.state_msg.enabled    != state_msg.enabled or
      self.state_msg.calibrated != state_msg.calibrated or
      self.state_msg.ready      != state_msg.ready or
      self.state_msg.moving     != state_msg.moving or
      self.state_msg.gripping   != state_msg.gripping or
      self.state_msg.missed     != state_msg.missed or
      self.state_msg.error      != state_msg.error or
      self._position_changed(self.state_msg.position, state_msg.position)  or
      self._force_changed(self.state_msg.force, state_msg.force)
      )

    if self.state_changed:
      self._format_state_report(state_msg)

    self.state_msg = state_msg

  def report(self):
    """ Report changed identity, properties and status
    """
    if self.identity_changed:
      print '-'*15, "Identity", '-'*15
      print self.identity_report
      self.identity_changed = False
    if self.properties_changed:
      print '-'*14, "Properties", '-'*14
      print self.properties_report
      self.properties_changed = False
    if self.state_changed:
      print '-'*16, "State" ,'-'*17
      print self.state_report
      self.state_changed = False


class BaxterController(object):
  """ Interface for a Baxter robot controller
  """
  def command(self, commands):
    """ generic command function
    Args:
      commands(dict{str:int}): a set of name/value pairs
        that forms a robot command
    """
    raise NotImplementedError()

  def record(self):
    """ records the last issued command or robot state
    """
    raise NotImplementedError()

  def report(self):
    """ reports status
    """
    raise NotImplementedError()


class GripperBaxterController(BaxterController):
  """ Controls the grippers on a Baxter Robot
  """
  def __init__(self):
    super(GripperBaxterController, self).__init__()
    self.left_gripper = GripperController('left')
    self.right_gripper = GripperController('right')
    self.grippers = { 
      'left' : self.left_gripper,
      'right' : self.right_gripper 
      }

  def command(self, commands):
    """ Handle a gripper command from a Mapper object.
        commands:  { 'command', { 'side' : side, 'arg': value, } }
    """
    for command, args in commands.iteritems():
      if args and ('side' in args):
        side = args['side']
        if side in self.grippers:
          gripper = self.grippers[side]
          gripper.handle_command(command, **args)
      else:
        print "Gripper command '%s', no side! args = %s" % (command, args)

  def report(self):
    """ Report gripper status
    """
    self.left_gripper.report()
    self.right_gripper.report()

