#!/usr/bin/env python

import rospy
from appctl.srv import Query
from appctl.msg import Mode
from statistics.msg import Session
import time


class AppctlSubscribeListener(rospy.SubscribeListener):
    def __init__(self, initial_state_method):
        self.initial_state_method = initial_state_method

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        rospy.loginfo("Peer subscribed to topic %s - sending last state" % topic_name)
        self.initial_state_method()

class AppctlSubscribeListener(rospy.SubscribeListener):
    def __init__(self, initial_state_method):
        self.initial_state_method = initial_state_method

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        rospy.loginfo("Peer subscribed to topic %s - sending last state" % topic_name)
        self.initial_state_method()

class AppController:
    """
    - provides service /appctl/query
        - provides mode state variable
        - provides default mode on startup
    - subscribes to /appctl/mode
        - provides Query.msg
    - publishes on /appctl/mode
        - TODO (wz) does a one-time only msg publish when launched
    - publishes on /statistics/session
    - provides initial_mode parameter
    """

    def __init__(self):
        self.node = self._init_node()
        self.mode = self._get_initial_mode()
        self.service = self._init_service()
        self.mode_publisher = self._init_mode_publisher()
        self.session_publisher = self._init_statistics_publisher()
        self.subscriber = self._init_subscriber()

    def run(self):
        rospy.loginfo("Starting Appcontroller service")
        self._publish_initial_state()
        while not rospy.is_shutdown():
            rospy.spin()
            pass
        pass

    def _publish_initial_state(self):
        rospy.loginfo("Publishing initial mode")
        mode_msg = Mode(mode=self._get_initial_mode())
        session_msg = Session(mode=self._get_initial_mode(), start_ts=int(time.time()))
        self.mode_publisher.publish(mode_msg)
        self.session_publisher.publish(session_msg)

    def _publish_state(self):
        rospy.loginfo("Publishing mode")
        mode_msg = Mode(mode=self.mode)
        session_msg = Session(mode=self.mode, start_ts=int(time.time()))
        self.mode_publisher.publish(mode_msg)
        self.session_publisher.publish(session_msg)

    def _get_mode(self):
        return self.mode

    def _set_mode(self, data):
        rospy.loginfo("Received new mode => {}".format(data.mode))
        self.mode = data.mode

    def _get_initial_mode(self):
        initial_mode = rospy.get_param('~initial_mode')
        return initial_mode

    def _init_node(self):
        rospy.init_node('appctl')

    def _init_subscriber(self):
        subscriber = rospy.Subscriber('/appctl/mode', Mode, self._set_mode)
        return subscriber

    def _init_mode_publisher(self):
        publisher = rospy.Publisher('/appctl/mode',
                                    Mode,
                                    queue_size=3,
                                    subscriber_listener=AppctlSubscribeListener(self._publish_state))
        return publisher

    def _init_statistics_publisher(self):
        publisher = rospy.Publisher('/statistics/session',
                                    Session,
                                    queue_size=3)
        return publisher

    def _process_service_request(self, req):
        """
        Callback for service requests. We always return mode
        """
        rospy.loginfo("Received Query service request {}".format(req))
        return self.mode

    def _init_service(self):
        service = rospy.Service('appctl/query',
                                Query,
                                self._process_service_request)
        return service


if __name__ == '__main__':
    try:
        AppController().run()
    except rospy.ROSInterruptException:
        pass

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
