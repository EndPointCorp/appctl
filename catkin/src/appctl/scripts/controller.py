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
    - publishes on /appctl/mode
        - does a one-time only msg publish when launched
    - provides initial_mode parameter
    """

    def __init__(self):
        self.node = self._init_node()
        self.mode = self._get_initial_mode()
        self.service = self._init_service()
        self.publisher = self._init_publisher()
        self.subscriber = self._init_subscriber()

        pass

    def run(self):
        rospy.loginfo("Starting Appcontroller service")
        while not rospy.is_shutdown():
            rospy.spin()
            pass
        pass

    def _get_mode(self):
        return self.mode

    def _set_mode(self, data):
        rospy.loginfo("Received new mode => {}".format(data.mode))
        self.mode = data.mode

    def _get_initial_mode(self):
        initial_mode = rospy.get_param('~initial_mode')
        return initial_mode

    def _init_node(self):
        rospy.init_node('appctl', anonymous=True)

    def _init_subscriber(self):
        subscriber = rospy.Subscriber('/appctl/mode', Mode, self._set_mode)
        return subscriber

    def _init_publisher(self):
        publisher = rospy.Publisher('/appctl/mode', Mode, queue_size = 3)
        rospy.loginfo("Publishing initial mode")
        msg = Mode()
        self.mode = self._get_initial_mode()
        msg.mode = self.mode
        publisher.publish(msg)
        return publisher

    def _init_service(self):
        service = rospy.Service('appctl/query',
                                Query,
                                self._get_mode())
        return service


if __name__ == '__main__':
    try:
        AppController().run()
    except rospy.ROSInterruptException:
        pass
