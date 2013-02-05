import os.path
import sys
import signal
import termios
import time
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
    print("Exiting...")
    self._done = True

  def done(self):
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
    self._done = False
    self.loop()

  def update(self):
    """ 
    Virtual update function.
    Called from self.loop() to mape input to control.
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
    self._setupBindings()

  def _setupBindings(self):
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
      Returns: a tuple containing a command label and a dictionary of args
      """
      def f(control_name):
        ret = None
        if buttons[control_name].down():
          ret = function()
        return ret or (None, None)
      return f

    def create_joybutton_function(buttons, joystick, cmd_label, cmd_args):
      """ function to create a joystick button control function
      Args: buttons
          func_label(string)
          args(dict) 
      Returns: a tuple containing a command label and a dictionary of args
      """
      def f(control_name):
        ret = None
        if buttons[control_name].down():
          print ("button %s pressed" % (control_name))
          cmd_args.update({'value': self.joystick.controls[control_name]})
          ret = (cmd_label, cmd_args)
        return ret or (None, None)
      return f

    def create_joystick_function(sticks, joystick, cmd_label, cmd_args):
      """ function to create a joystick stick control function
      Args: controls
          func_label(string)
          args(dict) 
      Returns: a tuple containing a command label and a dictionary of args
      """
      def f(control_name):
        ret = None
        if sticks[control_name].changed():
          value = self.joystick.controls[control_name]
          #print ("joystick %s = %f" % (control_name, value))
          cmd_args.update({'value': value})
          ret = (cmd_label, cmd_args)
        return ret or (None, None)
      return f


    self.bindings = {
      'function1':  create_command_function(buttons, self.stop),
      'function2':  create_command_function(buttons, self.stop),
      'rightBumper':  create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'left', 'position' : 100.0, 'velocity': 100.0}),
      'rightTrigger':  create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'left', 'position' : 0.0, 'velocity': 100.0}),
      'leftBumper':    create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'right', 'position' : 100.0, 'velocity': 100.0}),
      'leftTrigger':  create_joybutton_function(buttons, self.joystick, 'actuate',
                                              {'side': 'right', 'position' : 0.0, 'velocity': 100.0}),

      'rightStickHorz': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'left', 'position_scale': 100.0, 'velocity' : 30.0}),
      'leftStickHorz': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'right', 'position_scale': 100.0, 'velocity' : 30.0}),

      'rightStickVert': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'left', 'moving_force_inc': 2.0, 'holding_force_inc' : 2.0}),
      'leftStickVert': create_joystick_function(sticks, self.joystick, 'actuate',
                                              {'side': 'right', 'moving_force_inc': 2.0, 'holding_force_inc' : 2.0}),

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
      '[':   self.create_key_function('actuate', {'side': 'left',  'velocity' : 100.0, 
                                             'position' : 0.0}, 'left gripper:  close'),
      '{':   self.create_key_function('actuate', {'side': 'right', 'velocity' : 100.0, 
                                             'position' : 0.0},'right gripper:  close'),
      ']':   self.create_key_function('actuate', {'side': 'left',  'velocity' : 100.0, 
                                             'position' : 100.0}, 'left gripper:  open'),
      '}':   self.create_key_function('actuate', {'side': 'right', 'velocity' : 100.0, 
                                             'position' : 100.0}, 'right gripper:  open'),
      's':   self.create_key_function('actuate', {'side': 'left', 'velocity' : 0.0},
                                             'left gripper:  stop'),
      'S':   self.create_key_function('actuate', {'side': 'right', 'velocity' : 0.0},
                                             'right gripper:  stop'),

      '?': self.show_help,
      #' ': self.record,
      '\x1b': self.stop, #Escape... doesn't print.
    }

    self.show_help()

  def stop(self):
    super(KeyboardMapper, self).stop()

  def record(self):
    self.controller.record()

  def create_key_function(self, cmd_label, cmd_args, cmd_doc):
    """ function to create a keyboard control function
    cmd_label(string)
    cmd_args(dict) 
    cmd_doc(string)
    Returns: a tuple containing a command label and a dictionary of args
    """
    def f():
      return self.controller.command({ cmd_label : cmd_args})
    f.__doc__ = cmd_doc
    return f

  def getch(self, timeout=0.05):
    """Non-blocking getch.
    Returns an empty string if no character is available within the timeout
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
      tty.setraw(sys.stdin.fileno())
      [i, o, e] = select([sys.stdin.fileno()], [], [], 0.01)
      if i: 
        ch=sys.stdin.read(1)
      else: 
        ch=''
    except Exception as ex:
      print "getch", ex
      raise OSError
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

  def show_help(self):
    """show binding help"""
    print "="*20 + "gripper keyboard bindings " + "="*20
    for key, cmd in sorted(self.bindings.items(),key=lambda item: str.lower(item[0])):
      print "    " + str(key) + ": " + str(cmd.__doc__)

  def exec_binding(self, c):
    if not c:
      return
    if c in self.bindings:
      cmd = self.bindings[c]
      print cmd.__doc__
      cmd()
    else:
      print "unknown key: " + c
      print "press '?' for help"

  def update(self):
    self.controller.report()
    c = self.getch()
    self.exec_binding(c)


