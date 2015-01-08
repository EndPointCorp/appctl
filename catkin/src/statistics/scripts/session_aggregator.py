#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import sys
import json
from statistics.msg import Session
from statistics.srv import SessionQuery


class OutOfMemoryException(Exception):
    pass


class TooManyEventsException(Exception):
    pass


class SessionAggregator:
    """
    - provides session topic that acts as session events sink
    - provides session service for session event retrieval
    """

    def __init__(self):
        self.node = self._init_node()
        self.service = self._init_service()
        self.subscriber = self._init_subscriber()
        self.sessions = []
        self.max_events = rospy.get_param('~max_events', None)
        self.max_memory = rospy.get_param('~max_memory', '32000000')
        self.erase_flag = 0
        pass

    def _init_node(self):
        rospy.init_node('statistics')

    def _init_subscriber(self):
        subscriber = rospy.Subscriber('/statistics/session',
                                      Session,
                                      self._catch_session_event)
        return subscriber

    def _catch_session_event(self, data):
        self._append_event(data)

    def _append_event(self, event):
        """
        Handle the limits and append the event to the sessions storage
        """

        self._handle_erase_flag(event)

        if sys.getsizeof(self.max_memory) > self.max_memory:
            raise OutOfMemoryException

        if self.max_events and len(self.sessions) > self.max_events:
            raise TooManyEventsException("Number of events: %s, max: %s" % (len(self.sessions), self.max_events))

        rospy.loginfo("Got session => %s" % event.session)
        self.sessions.append(json.loads(event.session))
        pass

    def _erase_sessions(self):
        rospy.loginfo("Purging session events.")
        self.sessions = []
        self.erase_flag = 0

    def _handle_erase_flag(self, event):
        if self.erase_flag == 1:
            self._erase_sessions()
        if hasattr(event, 'erase'):
            if event.erase == 1:
                rospy.loginfo("Received `erase` flag. Session store will be erased upon next request")
                self.erase_flag = 1

    def _process_service_request(self, req):
        """
        Callback for service requests. We always return all sessions and
        check the `erase` flag
        """
        self._handle_erase_flag(req)
        return str(self.sessions)

    def _init_service(self):
        service = rospy.Service('statistics/session',
                                SessionQuery,
                                self._process_service_request)
        return service

    def run(self):
        rospy.loginfo("Starting session statistics service")
        while not rospy.is_shutdown():
            rospy.spin()
            pass
        pass

if __name__ == '__main__':
    try:
        SessionAggregator().run()
    except rospy.ROSInterruptException:
        pass
