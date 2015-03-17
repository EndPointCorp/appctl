#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import sys
import threading
from statistics.msg import Session
from statistics.srv import SessionQuery
from statistics.srv import SessionQueryResponse
from appctl.msg import Mode
from appctl.srv import Query

DEFAULT_MAX_EVENTS = None
DEFAULT_MAX_MEMORY = 32000000


class OutOfMemoryException(Exception):
    pass


class TooManyEventsException(Exception):
    pass


class SessionAggregator:
    """
    - provides session topic that acts as session events sink
    - provides session service for session event retrieval with possibility to retrieve only current session
    - this ROS node is internally operating on dict/json type and returns ROS msgs/srv
    """

    def __init__(self, max_events, max_memory, initial_mode):
        self.max_events = max_events
        self.max_memory = max_memory
        self.mode = initial_mode

        self.session_lock = threading.RLock()
        self.sessions = []
        self.session_fields = Session.__slots__
        self.session_mode = None
        self.session_application = None
        self._erase_sessions()

    def _filter_redundant_event(self, event):
        """ See if this event is redundantly starting a session with the same
            mode/app.
            """
        if event.start_ts != 0:
            if event.mode == self.session_mode and event.application == self.session_application:
                rospy.logdebug("Filtering out redundant {}/{} event at {}".format(event.mode, event.application, event.start_ts))
                rospy.logdebug("My session is {}/{}".format(self.session_mode, self.session_application))
                return True
            rospy.logdebug("Not filtering out {}/{} event at {}".format(event.mode, event.application, event.start_ts))
            rospy.logdebug("My session is {}/{}".format(self.session_mode, self.session_application))
            self.session_mode = event.mode
            self.session_application = event.application

        return False

    def handle_mode_change(self, msg):
        self.mode = msg.mode

    def route_event(self, event):
        """ For now - check some limits, handle flags and
            forward the event for appending
            """
        if self._filter_redundant_event(event):
            return

        with self.session_lock:
            self._append_event(event)

    def _check_max_memory(self):
        if self.max_memory is not None and sys.getsizeof(self.sessions) > self.max_memory:
            raise OutOfMemoryException

    def _check_max_events(self):
        if self.max_events is not None and len(self.sessions) >= self.max_events:
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
        self._check_max_memory()

        rospy.logdebug("Got session => %s" % event)
        rospy.logdebug("All stored sessions => %s" % self.sessions)

        event = self._handle_current_mode(event)

        if event.start_ts != 0:
            """ If there's "start_ts" set then we end the session and start new one"""
            self._check_max_events()
            self._end_previous_session(end_ts=event.start_ts)
            session = self._assemble_session(event)
            self.sessions.append(session)
        else:
            """ Otherwise we end the last session and clear state"""
            self._end_previous_session(end_ts=event.end_ts)
            self.session_mode = None
            self.session_application = None

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

        return session

    def _end_previous_session(self, end_ts):
        """
        - if session store has more than 1 session and new session event comes,
        end previous one using the start time of new one.
        - if "end_ts" is set, that means that no more sessions should be kept in session store
        """
        if len(self.sessions) >= 1 and self.sessions[-1]['end_ts'] == 0:
            rospy.loginfo("Ending previous session with time = %s" % end_ts)
            self.sessions[-1]['end_ts'] = end_ts

    def _erase_sessions(self):
        """
        Erase sessions - leave only open ones (those withoud "end_ts")
        """
        if len(self.sessions) > 0 and self.sessions[-1]['end_ts'] == 0:
            self.sessions = [self.sessions[-1]]
        else:
            self.sessions = []

    def _handle_erase_flag(self, event):
        if event.erase:
            rospy.loginfo("Erasing sessions")
            self._erase_sessions()

    def process_service_request(self, req):
        """
        Callback for service requests. We always return all sessions and
        check the `erase` flag
        """
        with self.session_lock:
            if req.current_only:
                current_session = self._get_current_session(self.sessions)
                rospy.loginfo("Currently open session: %s" % current_session)
                session_list = self._rewrite_response_to_ros(current_session)
            else:
                finished_sessions = self._get_finished_sessions(self.sessions)
                rospy.loginfo("Finished session(s) stored in memory: %s" % finished_sessions)
                session_list = self._rewrite_response_to_ros(finished_sessions)

            self._handle_erase_flag(req)

        return session_list

    def _rewrite_response_to_ros(self, sessions):
        """ Accepts list of dicts, returns SessionQueryResponse containing Sessions"""

        sessions_list = SessionQueryResponse()
        for session in sessions:
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
            if current_session['end_ts'] == 0:
                return [current_session]
        return []

    def _get_finished_sessions(self, sessions):
        """
        Filter out all sessions that don't have an 'end_ts' and return only
        those that were completed
        """
        finished_sessions = [s for s in sessions if (s['end_ts'] > 0)]
        assert isinstance(finished_sessions, list)

        return finished_sessions


def get_initial_mode():
    """ Sets initial mode (self.mode ivar) and starts first session """
    rospy.logdebug("Waiting for the /appctl/query service to become available")
    rospy.wait_for_service('/appctl/query')
    rospy.logdebug("appctl/query has become available")
    service_call = rospy.ServiceProxy('appctl/query', Query)
    mode = service_call().mode
    return mode


def main():
    rospy.init_node('statistics_session_aggregator')

    max_events = rospy.get_param('~max_events', DEFAULT_MAX_EVENTS)
    max_memory = rospy.get_param('~max_memory', DEFAULT_MAX_MEMORY)
    initial_mode = get_initial_mode()

    aggregator = SessionAggregator(
        max_events=max_events,
        max_memory=max_memory,
        initial_mode=initial_mode
    )

    rospy.Subscriber('/appctl/mode',
                     Mode,
                     aggregator.handle_mode_change)

    rospy.Subscriber('/statistics/session',
                     Session,
                     aggregator.route_event)

    rospy.Service('/statistics/session',
                  SessionQuery,
                  aggregator.process_service_request)

    rospy.spin()
