#!/usr/bin/env python

import rospy
from appctl.srv import Query
from appctl.msg import Mode


class AppController:
    """
    - provides service /appctl/query
        - provides mode state variable
        - provides default mode on startup
    - subscribes to /appctl/mode
        - provides Query.msg
    - provides initial_mode parameter
    """

    def __init__(self):
        self.default_mode = 'tactile'
        self.mode = self._get_initial_mode()
        self.service = self._init_service()
        pass

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
            pass
        pass

    def _get_current_mode(self):
        return self.mode

    def _get_initial_mode(self):
        initial_mode = rospy.get_param('~initial_mode', self.default_mode)
        return initial_mode

    def _init_node(self):
        rospy.init_node('appctl')

    def _init_subscribers(self):
        pass

    def _init_service(self):
        service = rospy.Service('appctl/query',
                                Query,
                                self._get_current_mode())
        return service


if __name__ == '__main__':
    try:
        AppController().run()
    except rospy.ROSInterruptException:
        pass
