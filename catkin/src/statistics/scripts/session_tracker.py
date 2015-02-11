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

    def handle_inactivity_duration(self, msg):
        duration = msg.data
        if not self.ended and duration > self.inactivity_timeout:
            self.ended = True
            self.publish_session_end()
            rospy.loginfo('ended')
        elif self.ended and duration <= self.inactivity_timeout:
            rospy.loginfo('active')
            self.publish_session_start()
            self.ended = False

    def publish_session_start(self):
        start_msg = Session(start_ts=int(time.time()))
        self.session_pub.publish(start_msg)

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

    ender = SessionEnder(inactivity_timeout, session_pub)

    inactivity_sub = rospy.Subscriber(
        '/portal_occupancy/interaction/inactive_duration',
        Duration,
        ender.handle_inactivity_duration
    )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
