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
        self.session_format = { 'start_ts': 0,
                                'end_ts': 0,
                                'app_name': 'appname'
                              }
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
        if self._validate_incoming_event(event):
            self.sessions.append(event.session)
        else:
            rospy.loginfo("Not appending event to session storage")
        pass

    def _validate_incoming_session(self, event):
        """
        Checks whether session has keys defined in self.session_format
        Returns None if any of the required keys are missing
        Returns `event` if event is valid
        """
        session = json.dumps(event.session)
        for key in self.session_format.iterkeys():
            if key not in session:
                rospy.loginfo("Received event session in wrong format - missing key: %s - please refer to the docs" % str(key))
                return None
        return session

    def _validate_incoming_event(self, event):
        if event.session:
            return self._validate_incoming_session(event)
        else:
            rospy.loginfo("Received event session in wrong format - please refer to the docs")
            return None

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
        rospy.loginfo("Returning aggregated sessions: %s" % self.sessions)
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
