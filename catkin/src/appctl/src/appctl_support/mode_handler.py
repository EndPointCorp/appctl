import rospy
from controller import BaseController

class ModeHandler():
  """
  Wraps a Controller and provides a mode message handler.
  """
  def __init__(self, modes, controller):
    assert isinstance(controller, BaseController)

    self.modes = modes
    self.controller = controller

  def handle_mode_msg(self, mode_msg):
    mode = mode_msg.mode
    rospy.logdebug('got mode {}'.format(mode))

    if mode in self.modes:
      self.controller.start()
    else:
      self.controller.stop()

  def shutdown(self):
    self.controller.stop()
