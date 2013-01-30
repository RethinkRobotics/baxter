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


