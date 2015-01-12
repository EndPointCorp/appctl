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
    - since sessions send only "start_ts", we need to end them automatically
        - populate previous (if exists) session's "end_ts" when new session arrives
        - if 'erase' flag is received - remove all sessions but last
    """

    def __init__(self):
        self.node = self._init_node()
        self.service = self._init_service()
        self.subscriber = self._init_subscriber()
        self.event_id = 0
        self.mandatory_fields = {'start_ts': 0}
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
        pass

        if sys.getsizeof(self.max_memory) > self.max_memory:
            raise OutOfMemoryException

        if self.max_events and len(self.sessions) > self.max_events:
            raise TooManyEventsException("Number of events: %s, max: %s" % (len(self.sessions), self.max_events))

        rospy.logdebug("Got session => %s" % event.session)
        if self._validate_incoming_event(event):
            session = json.loads(event.session)
            assert isinstance(session, dict)
            self.sessions.append(session)
            self._end_previous_session()
        else:
            rospy.logdebug("Not appending event to session storage")
        pass

    def _end_previous_session(self):
        """
        If session store has more than 1 session and new session event comes,
        end previous one using the start time of new one.
        """
        if len(self.sessions) >= 2:
            rospy.logdebug("Ending previous session with time = %s" % self.sessions[-1]['start_ts'])
            self.sessions[-2]['end_ts'] = self.sessions[-1]['start_ts']
        pass

    def _validate_incoming_session(self, event):
        """
        - Checks whether session has keys defined in self.mandatory_fields
        - Returns None if any of the required keys are missing
        - Returns `event` if event is valid
        """
        session = json.loads(event.session)

        for key in self.mandatory_fields.iterkeys():
            if key not in session:
                rospy.logwarn("Received event session in wrong format - missing key: %s - please refer to the docs" % str(key))
                return None
        # session['serial_number'] = self._get_event_id()
        # TODO(wz): a incremental serial number for sessions cleared after ROS restart
        assert isinstance(session, dict)
        return session

    def _validate_incoming_event(self, event):
        if event.session:
            return self._validate_incoming_session(event)
        else:
            rospy.logwarn("Received event session in wrong format - please refer to the docs")
            return None

    def _erase_sessions(self):
        """
        Check whether we have a last started session
            - if yes - erase all previous sessions and keep current
            - if not - just do nothing
        """

        assert isinstance(self.sessions, list)

        if self.sessions:
            current_session = self.sessions[-1]
            self.sessions = []
            assert isinstance(current_session, dict)
            rospy.logdebug("Purging session events but leaving the current one intact")
        else:
            rospy.logdebug("No session is running - keeping the session store intact")
            current_session = {}
            self.sessions = []

        rospy.logdebug("Leaving session %s of type %s in session store" % (current_session, type(current_session)))
        assert isinstance(current_session, dict)
        if current_session:
            self.sessions.append(current_session)
        self.erase_flag = 0
        pass

    def _get_event_id(self):
        self.event_id += 1
        return self.event_id

    def _handle_erase_flag(self, event):
        if self.erase_flag == 1:
            self._erase_sessions()
        if hasattr(event, 'erase'):
            if event.erase == 1:
                rospy.logdebug("Received `erase` flag. Session store will be purged upon next request")
                self.erase_flag = 1

    def _process_service_request(self, req):
        """
        Callback for service requests. We always return all sessions and
        check the `erase` flag
        """
        self._handle_erase_flag(req)
        rospy.loginfo("Currently open session: %s" % self.sessions[-1])
        finished_sessions = json.dumps(self._filter_finished_sessions(self.sessions))

        return finished_sessions

    def _filter_finished_sessions(self, sessions):
        """
        Filter out all sessions that don't have an 'end_ts' and return only
        those that were completed
        """
        finished_sessions = [s for s in sessions if 'end_ts' in s.iterkeys()]
        assert isinstance(finished_sessions, list)
        return finished_sessions

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
