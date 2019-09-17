import rospy
from .awesome import WindowManager
from .controller import BaseController


class WindowController(BaseController):
    """
    Controls visibility of a window.
    """
    def __init__(self, w_name=None, w_class=None, w_instance=None):
        self.window_manager = WindowManager(
            w_name,
            w_class,
            w_instance
        )
        self.w_name = w_name
        self.w_class = w_class
        self.w_instance = w_instance

    def start(self, *args, **kwargs):
        rospy.logdebug('showing window {} : {} : {}'.format(
            self.w_name, self.w_class, self.w_instance
        ))
        self.window_manager.show()

    def stop(self, *args, **kwargs):
        rospy.logdebug('hiding window {} : {} : {}'.format(
            self.w_name, self.w_class, self.w_instance
        ))
        self.window_manager.hide()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
