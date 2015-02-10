#!/usr/bin/env python

import rospy
from statistics.msg import Session
from std_msgs.msg import Duration
import time


class SessionEnder:
    def __init__(self, inactivity_timeout, session_pub):
        self.inactivity_timeout = inactivity_timeout
        self.session_pub = session_pub

        self.ended = False

    def handle_inactivity_duration(self, d):
        if not self.ended and d > self.inactivity_timeout:
            self.ended = True
            self.publish_session_end()
        elif self.ended and d <= self.inactivity_timeout:
            self.ended = False

    def publish_session_end(self):
        end_msg = Session(end_ts=int(time.time()))
        self.session_pub.publish(end_msg)


def main():
    rospy.init_node('session_ender')

    inactivity_timeout_s = int(rospy.get_param('~inactivity_timeout', 0))
    inactivity_timeout = rospy.Duration.from_sec(inactivity_timeout_s)

    session_pub = rospy.Publisher(
        '/statistics/session',
        Session,
        queue_size=2
    )
    inactivity_sub = rospy.Subscriber(
        '/portal_occupancy/interaction/inactivity_duration',
        Duration,
        self.handle_inactivity_duration
    )

    ender = SessionEnder(inactivity_timeout, session_pub)

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
