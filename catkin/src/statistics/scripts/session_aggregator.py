#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import sys
from statistics.msg import Session
from statistics.srv import SessionQuery
from statistics.srv import SessionQueryResponse
from appctl.msg import Mode
from appctl.srv import Query


class OutOfMemoryException(Exception):
    pass


class TooManyEventsException(Exception):
    pass


class SessionAggregator:
    """
    - provides session topic that acts as session events sink
    - provides session service for session event retrieval with possibility to retrieve only current session
    - this ROS node is internally operating on dict/json type and returns ROS msgs/srv
    TODO:
    - add "mode" to every session
    - make appctl send Session events with "start_ts" and "mode" filled in
    - make support for "application" attribute in session
    """

    def __init__(self):
        self.node = self._init_node()
        self.mode = self._get_initial_mode()
        self.max_events = rospy.get_param('~max_events', None)
        self.max_memory = rospy.get_param('~max_memory', '32000000')
        self.session_service = self._init_session_service()
        self.session_subscriber = self._init_session_subscriber()
        self.session_fields = Session.__slots__
        self.sessions = []
        self.erase_flag = 0
        pass

    def _init_node(self):
        rospy.init_node('statistics')

    def _get_initial_mode():
        rospy.logdebug("Waiting for the /appctl/query to become available")
        rospy.wait_for_service('appctl/query')
        service_call = rospy.ServiceProxy('appctl/query', Query)
        mode = service_call.mode
        return mode

    def _init_session_subscriber(self):
        subscriber = rospy.Subscriber('/statistics/session',
                                      Session,
                                      self._route_event)
        return subscriber

    def _wait_for_appctl_service(self):
        rospy.logdebug("Waiting for the /appctl/query service to become available")
        rospy.wait_for_service('appctl/query')
        rospy.logdebug("appctl/query has become available")
        pass

    def _route_event(self, event):
        """ For now - check some limits, handle flags and
            forward the event for appending
            """
        self._handle_erase_flag(event)
        self._check_max_memory()
        self._check_max_events()
        self._append_event(event)

    def _check_max_memory(self):
        if sys.getsizeof(self.max_memory) > self.max_memory:
            raise OutOfMemoryException
        pass

    def _check_max_events(self):
        if self.max_events and len(self.sessions) > self.max_events:
            raise TooManyEventsException("Number of events: %s, max: %s" % (len(self.sessions), self.max_events))

    def _handle_current_mode(self, event):
        """ Adds current mode do the session or sets current mode ivar"""
        if event.mode:
            self.mode = event.mode
        else:
            rospy.logdebug("Adding current mode (%s) to the session" % self.mode)
            event.mode = self.mode
        return event

    def _append_event(self, event):
        """
        - handle the limits
        - iterate over received session attributes and create a dict instance
        - fill in the missing attributes
        - if session's start_ts == 0, end session. If not - end session and start new one.
        """
        rospy.logdebug("Got session => %s" % event)
        rospy.logdebug("All stored sessions => %s" % self.sessions)

        event = self._handle_current_mode(event)

        if not event.start_ts == 0:
            """ If there's "start_ts" set then we end the session and start new one"""
            self._end_previous_session(end_ts=event.start_ts)
            session = self._assemble_session(event)
            self.sessions.append(session)
        else:
            """ Otherwise we end the last session """
            self._end_previous_session(end_ts=event.end_ts)
        pass

    def _assemble_session(self, event):
        """ First assemble session dict from what we've received,
        then get some additional params"""

        session = dict.fromkeys(self.session_fields)

        for attribute in self.session_fields:
            """ Try to assign received event's attributes to new session"""
            try:
                session[attribute] = event.__getattribute__(attribute)
            except Exception, e:
                rospy.logdebug("Session attributes dont match: %s" % e)
                pass

        return session

    def _end_previous_session(self, end_ts):
        """
        - if session store has more than 1 session and new session event comes,
        end previous one using the start time of new one.
        - if "end_ts" is set, that means that no more sessions should be kept in session store
        """
        if len(self.sessions) >= 1:
            rospy.loginfo("Ending previous session with time = %s" % end_ts)
            self.sessions[-1]['end_ts'] = end_ts
        pass

    def _erase_sessions(self):
        """
        Erase sessions - leave only open ones (those withoud "end_ts")
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
            if current_session['end_ts']:
                pass
            else:
                self.sessions.append(current_session)
        self.erase_flag = 0
        pass

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
        assert isinstance(self.sessions, list)
        if self.sessions:
            rospy.loginfo("Currently open session: %s" % self.sessions[-1])

        if req.current_only == 1:
            current_session = self._get_current_session(self.sessions)
            return current_session

        finished_sessions = self._get_finished_sessions(self.sessions)
        return self._rewrite_response_to_ros(finished_sessions)

    def _rewrite_response_to_ros(self, finished_sessions):
        sessions_list = SessionQueryResponse()
        for session in finished_sessions:
            s = Session()
            for attribute in Session.__slots__:
                s.__setattr__(attribute, session[attribute])
            sessions_list.sessions.append(s)
        rospy.logdebug("Returning session list: %s" % sessions_list)
        return sessions_list

    def _get_current_session(self, sessions):
        """
        Filter out all sessions that don't have an 'end_ts' and return only
        those that were completed
        """

        if sessions:
            current_session = sessions[-1]
            if current_session.end_ts == 0:
                return current_session
        return []

    def _get_finished_sessions(self, sessions):
        """
        Filter out all sessions that don't have an 'end_ts' and return only
        those that were completed
        """
        finished_sessions = [s for s in sessions if (s['end_ts'] > 0)]
        assert isinstance(finished_sessions, list)

        return finished_sessions

    def _init_session_service(self):
        service = rospy.Service('statistics/session',
                                SessionQuery,
                                self._process_service_request)
        return service

    def run(self):
        rospy.logdebug("Starting session statistics service")
        while not rospy.is_shutdown():
            rospy.spin()
            pass
        pass

if __name__ == '__main__':
    try:
        SessionAggregator().run()
    except rospy.ROSInterruptException:
        pass
