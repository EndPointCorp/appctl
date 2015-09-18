import rospy
from controller import BaseController
from appctl.msg import Mode
from appctl.srv import Query
from rospy import ServiceException


class ModeHandler():
    """
    Wraps a Controller and provides a mode message handler.
    """
    def __init__(self, modes, controller):
        assert isinstance(controller, BaseController)

        self.modes = modes
        self.controller = controller

    def _handle_mode_msg(self, mode_msg):
        mode = mode_msg.mode
        rospy.logdebug('got mode {}'.format(mode))

        if mode in self.modes:
            self.controller.start()
        else:
            self.controller.stop()

    def begin_handling_modes(self):
        rospy.Subscriber('/appctl/mode', Mode, self._handle_mode_msg)

        mode_query = rospy.ServiceProxy('/appctl/query', Query)
        try:
            mode_response = mode_query()
        except ServiceException:
            rospy.logdebug('/appctl/query service is not available')
        else:
            self._handle_mode_msg(mode_response)

    def shutdown(self):
        self.controller.stop()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
