#!/usr/bin/env python
PKG = 'statistics'
NAME = 'test_session_tracker'

import rospy
import unittest
import time

from portal_statistics.session_tracker import SessionEnder
from statistics.msg import Session
from std_msgs.msg import Duration

TEST_TIMEOUT = 20.0 # seconds


class MockPublisher:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class TestSessionEnder(unittest.TestCase):
    def setUp(self):
        rospy.init_node(NAME)
        timeout = rospy.Duration.from_sec(TEST_TIMEOUT)
        self.ender = SessionEnder(timeout, MockPublisher())

    def test_init_state(self):
        self.assertFalse(self.ender.ended,
            'SessionEnder must init unended')

    def test_session_end(self):
        pub = self.ender.session_pub

        over_duration = rospy.Duration.from_sec(TEST_TIMEOUT * 2)
        duration_msg = Duration(over_duration)
        self.ender.handle_inactivity_duration(duration_msg)

        self.assertTrue(self.ender.ended,
            'SessionEnder must end if duration greater than timeout')
        self.assertEqual(len(pub.msgs), 1,
            'SessionEnder must publish when session is ended')

        session_msg = pub.msgs[0]
        self.assertAlmostEqual(session_msg.end_ts, time.time(), delta=2,
            msg='Session end time must be just now')

    def test_session_start(self):
        pub = self.ender.session_pub
        self.ender.ended = True

        self.assertTrue(self.ender.ended,
            'Must test session start from an ended SessionEnder')

        under_duration = rospy.Duration.from_sec(TEST_TIMEOUT / 2)
        duration_msg = Duration(under_duration)
        self.ender.handle_inactivity_duration(duration_msg)

        self.assertFalse(self.ender.ended,
            'SessionEnder must start if duration less than timeout')
        self.assertEqual(len(pub.msgs), 1,
            'SessionEnder must publish when session is started')

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
            'SessionEnder must start if duration is exactly equal to timeout')


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestSessionEnder)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
