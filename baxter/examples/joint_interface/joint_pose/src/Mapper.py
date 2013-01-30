import signal
class Mapper(object):
  """ Interface class to map an input device to a controller
  """


  def __init__(self, controller):
    """ set up the ctrl-c handler for this input device
    Args:
      controller
    """
    self.controller = controller
    self.done = False
    signal.signal(signal.SIGINT, self.handleCtrlC)

  def handleCtrlC(self, signum, frame):
    """ ctrl-c handler
    """
    self.stop()

  def stop(self):
    self.done = True

  def loop(self):
    """ Virtual loop function
      function that loops while mapping
      input to control. Should end when self.done is True
    """
    raise NotImplementedError()


