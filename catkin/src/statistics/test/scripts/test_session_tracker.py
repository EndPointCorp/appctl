#!/usr/bin/env python
PKG = 'statistics'
NAME = 'test_session_tracker'

import rospy
import unittest
import time

from portal_statistics.session_tracker import SessionBreaker
from statistics.msg import Session
from std_msgs.msg import Duration

TEST_TIMEOUT = 20.0 # seconds


class MockSessionPublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class MockModePublisher:
    def __init__(self):
        pass

    def publish(self, msg):
        self.published_msg = msg


class MockModeResponse:
    def mode(self):
        return 'tactile'

class MockModeService:
    def __init__(self):
        pass

    def wait_for_service(self):
        pass

    def resolved_name(self):
        return '/appctl/mode'

    def __call__(self):
        return MockModeResponse()


class TestSessionBreaker(unittest.TestCase):
    def setUp(self):
        rospy.init_node(NAME)
        timeout = rospy.Duration.from_sec(TEST_TIMEOUT)
        fallback_mode = 'testing123'
        self.ender = SessionBreaker(inactivity_timeout=timeout,
                                    session_publisher=MockSessionPublisher(),
                                    fallback_mode=fallback_mode,
                                    fallback_publisher=MockModePublisher(),
                                    mode_service=MockModeService()
                                    )


    def test_init_state(self):
        self.assertFalse(self.ender.ended,
            'SessionBreaker must init unended')

    def test_session_end(self):
        pub = self.ender.session_publisher

        over_duration = rospy.Duration.from_sec(TEST_TIMEOUT * 2)
        duration_msg = Duration(over_duration)
        self.ender.handle_inactivity_duration(duration_msg)

        self.assertTrue(self.ender.ended,
            'SessionBreaker must end if duration greater than timeout')
        self.assertEqual(len(pub.msgs), 1,
            'SessionBreaker must publish when session is ended')

        session_msg = pub.msgs[0]
        self.assertAlmostEqual(session_msg.end_ts, time.time(), delta=2,
            msg='Session end time must be just now')

    def test_session_start(self):
        pub = self.ender.session_publisher
        self.ender.ended = True

        self.assertTrue(self.ender.ended,
            'Must test session start from an ended SessionBreaker')

        under_duration = rospy.Duration.from_sec(TEST_TIMEOUT / 2)
        duration_msg = Duration(under_duration)
        self.ender.handle_inactivity_duration(duration_msg)

        self.assertFalse(self.ender.ended,
            'SessionBreaker must start if duration less than timeout')
        self.assertEqual(len(pub.msgs), 1,
            'SessionBreaker must publish when session is started')

        session_msg = pub.msgs[0]
        self.assertAlmostEqual(session_msg.start_ts, time.time(), delta=2,
            msg='Session start time must be just now')

    def test_exact_duration(self):
        """
        What happens if we hit the inactivity duration on the nose?
        Counts as active.
        """
        self.ender.ended = True

        exact_duration = rospy.Duration.from_sec(TEST_TIMEOUT / 2)
        duration_msg = Duration(exact_duration)
        self.ender.handle_inactivity_duration(duration_msg)

        self.assertFalse(self.ender.ended,
            'SessionBreaker must start if duration is exactly equal to timeout')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestSessionBreaker)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
