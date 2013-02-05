"""
Keyboard, Joystick and File Mappers.
Mappers map input events to a controller.
"""
import sys
import signal
import termios
import tty
from select import select

import roslib
roslib.load_manifest('gripper_control')
import rospy


class Mapper(object):
  """ Interface class to map an input device to a controller
  """

  def __init__(self, controller, update_hz=100.0):
    """ set up the ctrl-c handler for this input device
    Args:
      controller
      update_hz
    """
    self.controller = controller
    self._done = False
    self.rate = rospy.Rate(update_hz)
    signal.signal(signal.SIGINT, self.handle_ctrl_c)

  def handle_ctrl_c(self, signum, frame):
    """ ctrl-c handler
    """
    self.stop()

  def stop(self):
    """ 
    Call stop to exit run().
    """
    print("Exiting...")
    self._done = True

  def done(self):
    """
    Return true if done (stop has been called).
    """
    return self._done

  def loop(self):
    """ 
      Timed loop.
      Calls self.update() to map input to control.
      Ends when self.stop() is called.
    """
    while not self.done():
      self.update()
      self.rate.sleep()

  def run(self):
    """
    Operate the mapper loop.
    """
    self._done = False
    self.loop()

  def update(self):
    """ 
    Virtual update function.
    Called from self.loop() to map input to control.
    """
    raise NotImplementedError()



class FileMapper(Mapper):
  """ CSV File mapper
  Maps input read from a csv file to robot control
  """

  def __init__(self, controller, filename):
    """ Maps csv file input to robot control
    Args:
      controller(BaxterController): a type of Baxter robot controller
      filename(str): path to csv file to read from
      rate(int): a rate in Hz to command each line of input at
    """
    super(FileMapper, self).__init__(controller)
    self.filename = filename
    self.start_time = rospy.Time.now()

  def time_stamp(self):
    """
    Returns the seconds elapsed since the start time.
    """
    diff = rospy.Time.now() - self.start_time
    return diff.to_sec()

  def loop(self):
    """ Loops through csv file
    Does not loop indefinitely, but only until the file is read
    and processed. Reads each line, split up in columns and
    formats each line into a controller command in the form of
    name/value pairs. Names come from the column headers
    first column is the time stamp
    """
    print("playing back %s" % (self.filename))
    with open(self.filename, 'r') as f:
      lines = f.readlines()
    keys = lines[0].rstrip().split(',')
    for values in lines[1:]:
      print(values)
      values = [float(x) for x in values.rstrip().split(',')]
      cmd = dict(zip(keys[1:], values[1:]))
      while (self.time_stamp() < values[0]):
        self.controller.command(cmd, False)
        rospy.sleep(0.01)

  def update(self):
    """ 
    Virtual update function.
    Called from self.loop() to map input to control.
    """
    # not used because loop is overridden
    raise NotImplementedError()


class JoystickMapper(Mapper):

  """ Maps joystick input to robot control
  """

  def __init__(self, controller, joystick):
    """ Maps joystick input to robot control
    Sets up the bindings
    Args:
      controller(BaxterController): a type of Baxter robot controller
      joystick(Joystick): Joystick handler
    """
    super(JoystickMapper, self).__init__(controller)
    self.joystick = joystick
    self.controls = {}
    self._setup_bindings()

  def _setup_bindings(self):
    """ private function to setup the bindings
    from generic joystick to robot command
    """

    buttons = self.joystick.create_button_changed_dict(
      'rightBumper',
      'leftBumper',
      'function1',
      'function2',
      'leftTrigger',
      'rightTrigger',
      'btnDown',
      'btnLeft',
      'btnRight',
      'dPadDown',
      'dPadLeft',
      'dPadRight',
      )

    sticks = self.joystick.create_stick_changed_dict(
      'leftStickHorz',
      'leftStickVert',
      'rightStickHorz',
      'rightStickVert',
      )

    def create_command_function(buttons, function):
      """ function to create a joystick button command function
      Args: buttons
            function
      Returns: a function that returns a joystick button response that takes no arguments
      """
      def command_function(control_name):
        """
        Args:  control_name(string)  Name of the control to respond to. 
        Returns: a the return value of a function executed in response to a joystick event
        """
        ret = None
        if buttons[control_name].down():
          ret = function()
        return ret or (None, None)
      return command_function

    def create_joybutton_function(buttons, joystick, cmd_label, cmd_args):
      """ function to create a joystick button control function
      Args: buttons(dict)   joystick button-changed dict
            joystick(Joystick) Joystick
            cmd_label(string) command label 
            cmd_args(dict)    command arguments
      Returns: a function that returns a joystick button response
      """
      def joybutton_function(control_name):
        """
        Args:  control_name(string)  Name of the control to respond to. 
        Returns: a tuple containing a command label and a dictionary of command arguments
        """
        ret = None
        if buttons[control_name].down():
          #print ("button %s pressed" % (control_name))
          cmd_args.update({'value': self.joystick.controls[control_name]})
          ret = (cmd_label, cmd_args)
        return ret or (None, None)
      return joybutton_function

    def create_joystick_function(sticks, joystick, cmd_label, cmd_args):
      """ function to create a joystick stick control function
      Args: sticks(dict)   Joystick control-changed dict
            joystick(Joystick) Joystick
            cmd_label(string) command label 
            cmd_args(dict)    command arguments
      Returns: a function that returns a joystick response
      """
      def joystick_function(control_name):
        """
        Args:  control_name(string)  Name of the control to respond to. 
        Returns: a tuple containing a command label and a dictionary of command arguments
        """
        ret = None
        if sticks[control_name].changed():
          value = self.joystick.controls[control_name]
          #print ("joystick %s = %f" % (control_name, value))
          cmd_args.update({'value': value})
          ret = (cmd_label, cmd_args)
        return ret or (None, None)
      return joystick_function

    self.bindings = {
      'function1':  create_command_function(buttons, self.stop),
      'function2':  create_command_function(buttons, self.stop),

      'rightBumper':  create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'left', 'position' : 100.0, 'velocity': 100.0}),
      'leftBumper':    create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'right', 'position' : 100.0, 'velocity': 100.0}),

      'rightTrigger':  create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'left', 'position' : 0.0, 'velocity': 100.0}),
      'leftTrigger':  create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'right', 'position' : 0.0, 'velocity': 100.0}),

      'rightStickHorz': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'left', 'position_scale': 100.0, 'velocity' : 30.0}),
      'leftStickHorz': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'right', 'position_scale': 100.0, 'velocity' : 30.0}),

      'rightStickVert': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'left', 'moving_force_scale': 100.0, 'holding_force_scale' : 100.0}),
      'leftStickVert': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'right', 'moving_force_scale': 100.0, 'holding_force_scale' : 100.0}),

      'btnLeft':   create_joybutton_function(buttons, self.joystick, 'reset', 
                                            {'side': 'left', 'reboot' : True}),
      'btnDown':   create_joybutton_function(buttons, self.joystick, 'calibrate', 
                                            {'side': 'left'}),
      'btnRight':   create_joybutton_function(buttons, self.joystick, 'actuate', 
                                            {'side': 'left', 'velocity' : 0.0}),

      'dPadLeft':   create_joybutton_function(buttons, self.joystick, 'reset', 
                                            {'side': 'right', 'reboot' : True}),
      'dPadDown':   create_joybutton_function(buttons, self.joystick, 'calibrate', 
                                            {'side': 'right'}),
      'dPadRight':   create_joybutton_function(buttons, self.joystick, 'actuate', 
                                            {'side': 'right', 'velocity' : 0.0}),
    }


  def update(self):
    """ Translate joystick input into robot control
    """
    self.controller.report()
    if self.joystick.new_data:
      commands = {}
      for control_name, control_function in self.bindings.items():
        (gripper_func, gripper_args) = control_function(control_name)
        if gripper_func != None:
          commands[gripper_func] = gripper_args
        self.controller.command(commands)


class KeyboardMapper(Mapper):
  """ class that listens to keypresses and sends associated robot joint commands """

  def __init__(self, controller):
    super(KeyboardMapper, self).__init__(controller)
    KeyboardMapper.show_help.__func__.__doc__ = "Show help."
    KeyboardMapper.stop.__func__.__doc__ = "(Esc) Exit."

    self.bindings = {
      'r':   self.create_key_function('reset', {'side': 'left', 'reboot' : True},
                                           'left gripper:  reset'),
      'R':   self.create_key_function('reset', {'side': 'right', 'reboot' : True},
                                           'right gripper:  reset'),
      'c':   self.create_key_function('calibrate', {'side': 'left'}, 'left gripper:  calibrate'),
      'C':   self.create_key_function('calibrate', {'side': 'right'}, 'right gripper:  calibrate'),
      'q':   self.create_key_function('actuate', {'side': 'left',  
                                             'position' : 0.0}, 'left gripper:  close'),
      'Q':   self.create_key_function('actuate', {'side': 'right', 
                                             'position' : 0.0},'right gripper:  close'),
      'w':   self.create_key_function('actuate', {'side': 'left',  
                                             'position' : 100.0}, 'left gripper:  open'),
      'W':   self.create_key_function('actuate', {'side': 'right', 
                                             'position' : 100.0}, 'right gripper:  open'),
      '[':   self.create_key_function('actuate', {'side': 'left',  'velocity' : 100.0, 
                                             'position' : 0.0}, 'left gripper:  close, 100% velocity'),
      '{':   self.create_key_function('actuate', {'side': 'right', 'velocity' : 100.0, 
                                             'position' : 0.0},'right gripper:  close, 100% velocity'),
      ']':   self.create_key_function('actuate', {'side': 'left',  'velocity' : 100.0, 
                                             'position' : 100.0}, 'left gripper:  open, 100% velocity'),
      '}':   self.create_key_function('actuate', {'side': 'right', 'velocity' : 100.0, 
                                             'position' : 100.0}, 'right gripper: open, 100% velocity'),
      's':   self.create_key_function('actuate', {'side': 'left', 'velocity' : 0.0},
                                             'left gripper:  stop'),
      'S':   self.create_key_function('actuate', {'side': 'right', 'velocity' : 0.0},
                                             'right gripper:  stop'),
      'z':   self.create_key_function('actuate', {'side': 'left', 'dead_zone_inc' : -1.0},
                                             'left gripper:  decrease dead zone'),
      'Z':   self.create_key_function('actuate', {'side': 'right', 'dead_zone_inc' : -1.0},
                                             'right gripper:  decrease dead zone'),
      'x':   self.create_key_function('actuate', {'side': 'left', 'dead_zone_inc' : 1.0},
                                             'left gripper:  increase dead zone'),
      'X':   self.create_key_function('actuate', {'side': 'right', 'dead_zone_inc' : 1.0},
                                             'right gripper:  increase dead zone'),
      'f':   self.create_key_function('actuate', {'side': 'left', 'moving_force_inc' : -5.0},
                                             'left gripper:  decrease moving force'),
      'F':   self.create_key_function('actuate', {'side': 'right', 'moving_force_inc' : -5.0},
                                             'right gripper:  decrease moving force'),
      'g':   self.create_key_function('actuate', {'side': 'left', 'moving_force_inc' : 5.0},
                                             'left gripper:  increase moving force'),
      'G':   self.create_key_function('actuate', {'side': 'right', 'moving_force_inc' : 5.0},
                                             'right gripper:  increase moving force'),
      'h':   self.create_key_function('actuate', {'side': 'left', 'holding_force_inc' : -5.0},
                                             'left gripper:  decrease holding force'),
      'H':   self.create_key_function('actuate', {'side': 'right', 'holding_force_inc' : -5.0},
                                             'right gripper:  decrease holding force'),
      'j':   self.create_key_function('actuate', {'side': 'left', 'holding_force_inc' : 5.0},
                                             'left gripper:  increase holding force'),
      'J':   self.create_key_function('actuate', {'side': 'right', 'holding_force_inc' : 5.0},
                                             'right gripper:  increase holding force'),
      'v':   self.create_key_function('actuate', {'side': 'left', 'velocity_inc' : -5.0},
                                             'left gripper:  decrease velocity'),
      'V':   self.create_key_function('actuate', {'side': 'right', 'velocity_inc' : -5.0},
                                             'right gripper:  decrease velocity'),
      'b':   self.create_key_function('actuate', {'side': 'left', 'velocity_inc' : 5.0},
                                             'left gripper:  increase velocity'),
      'B':   self.create_key_function('actuate', {'side': 'right', 'velocity_inc' : 5.0},
                                             'right gripper:  increase velocity'),



      '?': self.show_help,
      #' ': self.record,
      '\x1b': self.stop, #Escape... doesn't print.
    }

    self.show_help()

  def stop(self):
    """ Stop.  Results in program exit.
    """
    super(KeyboardMapper, self).stop()

  def record(self):
    """ Start recording keystrokes
    """
    self.controller.record()

  def create_key_function(self, cmd_label, cmd_args, cmd_doc):
    """ function to create a keyboard control function
    cmd_label(string)  command label
    cmd_args(dict)     command arguments
    cmd_doc(string)    key function __doc__ override
    Returns: a function that executes a controller command
    """
    def key_function():
      """
      Returns: the result of a controller command
      """
      return self.controller.command({ cmd_label : cmd_args})
    key_function.__doc__ = cmd_doc
    return key_function

  def getch(self, timeout=0.01):
    """Non-blocking getch.
    Returns an empty string if no character is available within the timeout
    """
    fileno = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fileno)
    try:
      tty.setraw(fileno)
      [i, _, _] = select([fileno], [], [], timeout)
      if i: 
        chr_ = sys.stdin.read(1)
      else: 
        chr_ = ''
    except Exception as ex:
      print "getch", ex
      raise OSError
    finally:
      termios.tcsetattr(fileno, termios.TCSADRAIN, old_settings)
    return chr_

  def show_help(self):
    """show binding help"""
    print "="*20 + "gripper keyboard bindings " + "="*20
    for key, cmd in sorted(self.bindings.items(), key=lambda item: str.lower(item[0])):
      print "    " + str(key) + ": " + str(cmd.__doc__)

  def exec_binding(self, chr_):
    """ Execute the function bound to the given key.
    """
    if not chr_:
      return
    if chr_ in self.bindings:
      cmd = self.bindings[chr_]
      print cmd.__doc__
      cmd()
    else:
      print "unknown key: " + chr_
      print "press '?' for help"

  def update(self):
    """ Update the controller.   Execute the next keyboard key binding.
    """
    self.controller.report()
    chr_ = self.getch()
    self.exec_binding(chr_)


