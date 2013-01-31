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


