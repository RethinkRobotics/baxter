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
    self.bindings = {}
    JoystickMapper.show_help.__func__.__doc__ = "Show help"
    JoystickMapper.stop.__func__.__doc__ = "Exit"
    self._setup_bindings()
    self.show_help()

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
      'btnUp',
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

    def create_command_function(buttons, function, doc):
      """ function to create a joystick button command function
      Args: buttons
            function
            doc(string)   Doc string
      Returns: a function that returns a joystick button response that takes no arguments
      """
      def command_function(control_name):
        """
        Args:  control_name(string)  Name of the control to respond to.
        Returns: the return value of a function executed in response to a joystick event
        """
        ret = None
        if buttons[control_name].down():
          ret = function()
        return ret or (None, None)
      command_function.__doc__ = doc
      return command_function

    def create_joybutton_function(buttons, joystick, cmd_label, cmd_args, cmd_doc):
      """ function to create a joystick button control function
      Args: buttons(dict)   joystick button-changed dict
            joystick(Joystick) Joystick
            cmd_label(string) command label
            cmd_args(dict)    command arguments
            cmd_doc(string)   doc string
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
          cmd_args.update({'value': joystick.controls[control_name]})
          ret = (cmd_label, cmd_args)
        return ret or (None, None)
      joybutton_function.__doc__ = cmd_args['side'] + ":" + cmd_doc
      return joybutton_function

    def create_joystick_function(sticks, joystick, cmd_label, cmd_args, cmd_doc):
      """ function to create a joystick stick control function
      Args: sticks(dict)   Joystick control-changed dict
            joystick(Joystick) Joystick
            cmd_label(string) command label
            cmd_args(dict)    command arguments
            cmd_doc(string)   doc string
      Returns: a function that returns a joystick response
      """
      def joystick_function(control_name):
        """
        Args:  control_name(string)  Name of the control to respond to.
        Returns: a tuple containing a command label and a dictionary of command arguments
        """
        ret = None
        if sticks[control_name].changed():
          value = joystick.controls[control_name]
          #print ("joystick %s = %f" % (control_name, value))
          cmd_args.update({'value': value})
          ret = (cmd_label, cmd_args)
        return ret or (None, None)
      joystick_function.__doc__ = cmd_args['side'] + ":" + cmd_doc
      return joystick_function

    cjbf = create_joybutton_function
    cjsf = create_joystick_function


    self.bindings = {
      'function1':  create_command_function(buttons, self.stop, "Exit."),
      'function2':  create_command_function(buttons, self.stop, "Exit."),
      'btnUp'    :  create_command_function(buttons, self.show_help, "Show help."),

      'rightBumper':  cjbf(buttons, self.joystick, 'actuate',
                          {'side': 'left', 'position' : 100.0, 'velocity': 100.0}, 
                           'Open'),
      'leftBumper':   cjbf(buttons, self.joystick, 'actuate',
                          {'side': 'right', 'position' : 100.0, 'velocity': 100.0}, 
                           'Open'),

      'rightTrigger': cjbf(buttons, self.joystick, 'actuate',
                          {'side': 'left', 'position' : 0.0, 'velocity': 100.0}, 
                           'Close'),
      'leftTrigger':  cjbf(buttons, self.joystick, 'actuate',
                          {'side': 'right', 'position' : 0.0, 'velocity': 100.0}, 
                           'Close'),

      'rightStickHorz': cjsf(sticks, self.joystick, 'actuate',
                             {'side': 'left', 'position_scale': 100.0,
                              'velocity' : 30.0}, 'Control position'),
      'leftStickHorz': cjsf(sticks, self.joystick, 'actuate',
                            {'side': 'right', 'position_scale': 100.0,
                             'velocity' : 30.0}, 'Control position'),

      'rightStickVert': cjsf(sticks, self.joystick, 'actuate',
                             {'side': 'left', 'moving_force_scale': 100.0,
                              'holding_force_scale' : 100.0}, 
                             'Control force'),
      'leftStickVert': cjsf(sticks, self.joystick, 'actuate',
                            {'side': 'right', 'moving_force_scale': 100.0,
                             'holding_force_scale' : 100.0}, 
                            'Control force'),

      'btnLeft':   cjbf(buttons, self.joystick, 'reset',
                       {'side': 'left', 'reboot' : True}, 'Reset'),
      'btnDown':   cjbf(buttons, self.joystick, 'calibrate',
                       {'side': 'left'}, 'Calibrate'),
      'btnRight':   cjbf(buttons, self.joystick, 'actuate',
                        {'side': 'left', 'velocity' : 0.0}, 'Stop'),

      'dPadLeft':   cjbf(buttons, self.joystick, 'reset',
                        {'side': 'right', 'reboot' : True}, 'Reset'),
      'dPadDown':   cjbf(buttons, self.joystick, 'calibrate',
                        {'side': 'right'}, 'Calibrate'),
      'dPadRight':   cjbf(buttons, self.joystick, 'actuate',
                         {'side': 'right', 'velocity' : 0.0}, 'Stop'),
      }

  def show_help(self):
    """show binding help"""
    neither = []
    left = []
    right = []
    for control_name, control_function in self.bindings.items():
      side, _, doc = str(control_function.__doc__).partition(':')
      hlp = "    %-14s:  %s" % ( str(control_name), (doc if doc else side))
      if (side == 'left'):
        left.append(hlp)
      elif (side == 'right'):
        right.append(hlp)
      else:
        neither.append(hlp)

    print "*"*7 + "gripper game controller bindings " + "*"*7
    for hlp in sorted(neither):
      print hlp
    print
    print "Left Gripper:"
    for hlp in sorted(left):
      print hlp
    print
    print "Right Gripper:"
    for hlp in sorted(right):
      print hlp
    print "*"*40


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
    ckf = self.create_key_function
    self.bindings = {
      'r':   ckf('reset', {'side': 'left', 'reboot' : True},
                 'left:  reset'),
      'R':   ckf('reset', {'side': 'right', 'reboot' : True},
                 'right:  reset'),
      'c':   ckf('calibrate', {'side': 'left'}, 'left:  calibrate'),
      'C':   ckf('calibrate', {'side': 'right'}, 'right:  calibrate'),
      'q':   ckf('actuate', {'side': 'left',
                             'position' : 0.0}, 'left:  close'),
      'Q':   ckf('actuate', {'side': 'right',
                             'position' : 0.0},'right:  close'),
      'w':   ckf('actuate', {'side': 'left',
                             'position' : 100.0}, 'left:  open'),
      'W':   ckf('actuate', {'side': 'right',
                             'position' : 100.0}, 'right:  open'),
      '[':   ckf('actuate', {'side': 'left',  'velocity' : 100.0,
                             'position' : 0.0}, 'left:  close, 100% velocity'),
      '{':   ckf('actuate', {'side': 'right', 'velocity' : 100.0,
                             'position' : 0.0},'right:  close, 100% velocity'),
      ']':   ckf('actuate', {'side': 'left',  'velocity' : 100.0,
                             'position' : 100.0}, 'left:  open, 100% velocity'),
      '}':   ckf('actuate', {'side': 'right', 'velocity' : 100.0,
                             'position' : 100.0}, 'right:  open, 100% velocity'),
      's':   ckf('actuate', {'side': 'left', 'velocity' : 0.0}, 'left:  stop'),
      'S':   ckf('actuate', {'side': 'right', 'velocity' : 0.0}, 'right:  stop'),
      'z':   ckf('actuate', {'side': 'left', 'dead_zone_inc' : -1.0},
                 'left:  decrease dead zone'),
      'Z':   ckf('actuate', {'side': 'right', 'dead_zone_inc' : -1.0},
                 'right:  decrease dead zone'),
      'x':   ckf('actuate', {'side': 'left', 'dead_zone_inc' : 1.0},
                 'left:  increase dead zone'),
      'X':   ckf('actuate', {'side': 'right', 'dead_zone_inc' : 1.0},
                 'right:  increase dead zone'),
      'f':   ckf('actuate', {'side': 'left', 'moving_force_inc' : -5.0},
                 'left:  decrease moving force'),
      'F':   ckf('actuate', {'side': 'right', 'moving_force_inc' : -5.0},
                 'right:  decrease moving force'),
      'g':   ckf('actuate', {'side': 'left', 'moving_force_inc' : 5.0},
                 'left:  increase moving force'),
      'G':   ckf('actuate', {'side': 'right', 'moving_force_inc' : 5.0},
                 'right:  increase moving force'),
      'h':   ckf('actuate', {'side': 'left', 'holding_force_inc' : -5.0},
                 'left:  decrease holding force'),
      'H':   ckf('actuate', {'side': 'right', 'holding_force_inc' : -5.0},
                 'right:  decrease holding force'),
      'j':   ckf('actuate', {'side': 'left', 'holding_force_inc' : 5.0},
                 'left:  increase holding force'),
      'J':   ckf('actuate', {'side': 'right', 'holding_force_inc' : 5.0},
                 'right:  increase holding force'),
      'v':   ckf('actuate', {'side': 'left', 'velocity_inc' : -5.0},
                 'left:  decrease velocity'),
      'V':   ckf('actuate', {'side': 'right', 'velocity_inc' : -5.0},
                 'right:  decrease velocity'),
      'b':   ckf('actuate', {'side': 'left', 'velocity_inc' : 5.0},
                 'left:  increase velocity'),
      'B':   ckf('actuate', {'side': 'right', 'velocity_inc' : 5.0},
                 'right:  increase velocity'),
      '?':   self.show_help,
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

  def getch(timeout=0.01):
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
  getch = staticmethod(getch)

  def show_help(self):
    """show binding help"""
    neither = []
    left = []
    right = []
    for key, cmd in sorted(self.bindings.items(), key=lambda item: item[1].__doc__[0]+str.lower(item[0])):
      side, _, doc = str(cmd.__doc__).partition(':')
      hlp = "    " + str(key) + ": " + (doc if doc else side)
      if (side == 'left'):
        left.append(hlp)
      elif (side == 'right'):
        right.append(hlp)
      else:
        neither.append(hlp)

    print "*"*7 + "gripper keyboard bindings " + "*"*7
    for hlp in neither:
      print hlp
    print
    print "Left Gripper:"
    for hlp in left:
      print hlp
    print
    print "Right Gripper:"
    print "    Shifted keys control the right gripper."
    for hlp in right:
      print hlp
    print "*"*40

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
