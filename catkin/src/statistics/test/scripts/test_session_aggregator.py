#!/usr/bin/env python
PKG = 'statistics'
NAME = 'test_session_aggregator'

import sys
import time
import unittest

from portal_statistics import session_aggregator
from portal_statistics.session_aggregator import SessionAggregator
from statistics.msg import Session
from statistics.srv import SessionQuery, SessionQueryResponse
from appctl.srv import Query, QueryResponse

TEST_MAX_EVENTS = session_aggregator.DEFAULT_MAX_EVENTS
TEST_MAX_MEMORY = session_aggregator.DEFAULT_MAX_MEMORY
TEST_INITIAL_MODE = 'tactile'


class TestSessionAggregator(unittest.TestCase):
    def setUp(self):
        self.aggregator = SessionAggregator(
            max_events=TEST_MAX_EVENTS,
            max_memory=TEST_MAX_MEMORY,
            initial_mode=TEST_INITIAL_MODE
        )

    def get_sessions(self, erase=False, current_only=False):
        """Fetches sessions from the aggregator."""
        # XXX: passing **kwargs doesn't work for some reason?
        req = SessionQuery()
        req.erase = erase
        req.current_only = current_only
        return self.aggregator.process_service_request(req).sessions

    def has_finished_session(self):
        """Checks for finished sessions on the aggregator service."""
        if len(self.get_sessions(current_only=False)) > 0:
            return True
        return False

    def has_current_session(self):
        """Checks for unfinished sessions on the aggregator service."""
        if len(self.get_sessions(current_only=True)) > 0:
            return True
        return False

    def has_sessions(self):
        """Checks for any sessions on the aggregator service."""
        return self.has_finished_session() or self.has_current_session()

    def submit_session(self, *args, **kwargs):
        """Sends a session event to the aggregator."""
        msg = Session(**kwargs)
        self.aggregator.route_event(msg)

    def test_init_state(self):
        self.assertFalse(self.has_sessions(),
            'Must init with no sessions')

    def test_end_first(self):
        self.submit_session(end_ts = 10000)

        self.assertFalse(self.has_sessions(),
            'Must have no sessions after initial end')

    def test_start_and_end(self):
        test_start_ts = 9000
        test_end_ts = 10000

        self.submit_session(start_ts=test_start_ts)

        self.assertTrue(self.has_current_session(),
            'Must have a current session')

        self.submit_session(end_ts=test_end_ts)

        finished_sessions = self.get_sessions()
        self.assertTrue(len(finished_sessions) == 1,
            'Must have one finshed session after end')

        self.assertFalse(self.has_current_session(),
            'Must not have a current session after end')

        session = finished_sessions[0]
        self.assertEqual(session.mode, TEST_INITIAL_MODE,
            'Session must have correct mode')
        self.assertEqual(session.application, '',
            'Session must have no application')
        self.assertEqual(session.start_ts, test_start_ts,
            'Session must have correct start_ts')
        self.assertEqual(session.end_ts, test_end_ts,
            'Session must have correct end_ts')

    def test_occupancy_trigger(self):
        test_start_ts = 9000
        test_end_ts = 10000

        self.submit_session(start_ts=test_start_ts, occupancy_triggered=True)
        self.submit_session(end_ts=test_end_ts)

        finished_sessions = self.get_sessions()
        session = finished_sessions[0]

        self.assertTrue(session.occupancy_triggered,
            'Session must have occupancy_triggered flag')

    def test_multi_sessions(self):
        num_cases = 20
        def make_case(i):
            return {
                'start_ts': i * 1000 + 100,
                'end_ts': i * 1000 + 900
            }
        cases = map(make_case, range(num_cases))

        for i in range(num_cases):
            case = cases[i]

            self.submit_session(start_ts = case['start_ts'])
            self.assertTrue(self.has_current_session(),
                'Must have a current session [i={}]'.format(i))

            self.submit_session(end_ts = case['end_ts'])
            self.assertFalse(self.has_current_session(),
                'Must not have a current session [i={}]'.format(i))

            sessions = self.get_sessions()
            self.assertEqual(len(sessions), i + 1,
                'Must have [{}] finished sessions'.format(i + 1))

            ### Check all previous sessions for consistency
            for j in range(i):
                prev_case = cases[j]
                prev_session = sessions[j]
                self.assertEqual(prev_case['start_ts'], prev_session.start_ts,
                    'Previous start time corrupted [i={}, j={}]'.format(i, j))
                self.assertEqual(prev_case['end_ts'], prev_session.end_ts,
                    'Previous end time corrupted [i={}, j={}]'.format(i, j))

    def test_erase_finished_sessions(self):
        self.submit_session(start_ts=1000)
        self.submit_session(end_ts=2000)

        self.get_sessions(erase=True)

        self.assertFalse(self.has_finished_session(),
            'Must not have a finished session after erase')

    def test_erase_preserve_current_session(self):
        final_start_ts = 3000
        final_end_ts = 4000
        self.submit_session(start_ts=1000)
        self.submit_session(end_ts=2000)
        self.submit_session(start_ts=final_start_ts)

        self.get_sessions(erase=True)

        self.assertTrue(self.has_current_session(),
            'Must have a current session after erase')
        self.assertFalse(self.has_finished_session(),
            'Must not have a finished session after erase')

        self.submit_session(end_ts=final_end_ts)
        self.assertTrue(self.has_finished_session(),
            'Must have a finished session continuing through erase')

        session = self.get_sessions()[0]
        self.assertEqual(session.start_ts, final_start_ts,
            'Final Session must have correct start_ts')
        self.assertEqual(session.end_ts, final_end_ts,
            'Final Session must have correct end_ts')

    def test_back_to_back_sessions(self):
        self.submit_session(start_ts=1000, mode='qwerty')
        self.submit_session(start_ts=1200, mode='asdf')
        self.submit_session(start_ts=1400, mode='hjkl')
        self.submit_session(end_ts=1500)

        sessions = self.get_sessions()
        self.assertEqual(len(sessions), 3,
            'Must have three sessions after back to back tests')
        self.assertEqual(sessions[0].start_ts, 1000)
        self.assertEqual(sessions[0].end_ts, 1200)
        self.assertEqual(sessions[1].start_ts, 1200)
        self.assertEqual(sessions[1].end_ts, 1400)
        self.assertEqual(sessions[2].start_ts, 1400)
        self.assertEqual(sessions[2].end_ts, 1500)

    def test_filter_redundant_mode(self):
        test_mode = 'asdf'
        self.submit_session(start_ts=1000, mode=test_mode)
        self.submit_session(start_ts=1200, mode=test_mode)

        self.assertFalse(self.has_finished_session(),
            'Must not have a finished session after redundant mode start')

        self.submit_session(end_ts=1500)
        self.assertEqual(len(self.get_sessions()), 1,
            'Must have only one session after redundant mode start and finish')

    def test_filter_redundant_application(self):
        test_application = 'asdf'
        self.submit_session(start_ts=1000, application=test_application)
        self.submit_session(start_ts=1200, application=test_application)

        self.assertFalse(self.has_finished_session(),
            'Must not have a finished session after redundant app start')

        self.submit_session(end_ts=1500)
        self.assertEqual(len(self.get_sessions()), 1,
            'Must have only one session after redundant app start and finish')

    def test_max_events(self):
        max_events = 5
        self.aggregator = SessionAggregator(
            max_events=max_events,
            max_memory=None,
            initial_mode=TEST_INITIAL_MODE
        )
        for i in range(max_events):
            self.submit_session(start_ts=i * 1000 + 100)
            self.submit_session(end_ts=i * 1000 + 900)

        with self.assertRaises(session_aggregator.TooManyEventsException):
            self.submit_session(start_ts=max_events * 1000)

    def test_max_memory(self):
        ### Find typical size of Session and set max in the ballpark of size*N
        desired_events = 5
        exemplar = Session(mode=TEST_INITIAL_MODE, start_ts=1, end_ts=2)
        session_size = sys.getsizeof(exemplar)
        max_memory = session_size * desired_events
        self.aggregator = SessionAggregator(
            max_events=None,
            max_memory=max_memory,
            initial_mode=TEST_INITIAL_MODE
        )

        with self.assertRaises(session_aggregator.OutOfMemoryException):
            ### The size estimate is not exact, so overshoot to trigger OOM
            for i in range(desired_events * 10):
                self.submit_session(start_ts=i * 1000 + 100)
                self.submit_session(end_ts=i * 1000 + 900)

                session_list_size = sys.getsizeof(self.aggregator.sessions)
                self.assertFalse(session_list_size > max_memory,
                    'Session list size must not exceed max_memory')


if __name__ == '__main__':
    import rospy
    import rostest
    rospy.init_node(NAME, log_level=rospy.DEBUG)
    rostest.rosrun(PKG, NAME, TestSessionAggregator)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
