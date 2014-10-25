import rospy
from proc_controller import ProcController

class ModeHandler():
  """
  Wraps an AppController and provides a mode message handler.
  """
  def __init__(self, modes, cmd):
    self.modes = modes
    self.controller = ProcController(cmd)

  def handle_mode_msg(self, mode_msg):
    mode = mode_msg.mode
    rospy.logdebug('got mode {}'.format(mode))

    if mode in self.modes:
      self.controller.start()
    else:
      self.controller.stop()

  def shutdown(self):
    self.controller.stop()
