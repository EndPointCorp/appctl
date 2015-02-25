#!/usr/bin/env python

import rospy
import time
from statistics.msg import Session
from std_msgs.msg import Duration
from statistics.srv import SessionQuery

DEFAULT_SESSION_TIMEOUT = 20.0 # seconds


class SessionBreaker:
    """
    - watches the inactivity duration and sends Session:end_ts=0 to statistics aggregator
    to end current session if it exists
    - caches current_session in self.cached_session (Session type) so it can unpause it after proximity sensor is triggered
    """
    def __init__(self, inactivity_timeout, session_pub):
        self.inactivity_timeout = inactivity_timeout
        self.current_session_service = self._wait_for_service()
        self.session_pub = session_pub
        self.cached_session = None

        self.ended = False

    def _wait_for_service(self):
        rospy.logdebug("Waiting for the /statistics/session service to become available")
        rospy.wait_for_service('statistics/session')
        rospy.logdebug("Statistics aggretator service has become available")
        pass

    def _get_current_session(self):
        service_call = rospy.ServiceProxy('statistics/session', SessionQuery)
        response = service_call(erase=False, current_only=True)
        rospy.logdebug("Received response from service: %s" % response)
        current_session = response.sessions[0]
        return current_session

    def handle_inactivity_duration(self, msg):
        duration = msg.data
        if not self.ended and duration > self.inactivity_timeout:
            self.ended = True
            self._cache_current_session()
            self.publish_session_end()
            rospy.loginfo('ended')
        elif self.ended and duration <= self.inactivity_timeout:
            rospy.loginfo('active')
            self.publish_session_continue()
            self.ended = False

    def publish_session_continue(self):
        """
        Get last known session and rewrite start_ts and end_ts
        to make it continue. Also make the prox_sensor_triggered equals True
        """
        if self.cached_session:
            start_msg = self._get_cached_session()
            start_msg.start_ts = int(time.time())
            start_msg.end_ts = 0
            start_msg.prox_sensor_triggered = 1
        else:
            start_msg = Session(
                                start_ts=int(time.time()),
                                prox_sensor_triggered=1
                                )

        self.session_pub.publish(start_msg)

    def publish_session_end(self):
        end_msg = Session(end_ts=int(time.time()))
        self.session_pub.publish(end_msg)

    def _cache_current_session(self):
        self.current_session = self._get_current_session()

    def _get_cached_session(self):
        return self.cached_session


def main():
    rospy.init_node('session_ender')

    inactivity_timeout_s = rospy.get_param(
        '~inactivity_timeout',
        DEFAULT_SESSION_TIMEOUT
    )
    inactivity_timeout = rospy.Duration.from_sec(float(inactivity_timeout_s))


    session_pub = rospy.Publisher(
        '/statistics/session',
        Session,
        queue_size=2
    )

    ender = SessionBreaker(inactivity_timeout, session_pub)

    inactivity_sub = rospy.Subscriber(
        '/portal_occupancy/interaction/inactive_duration',
        Duration,
        ender.handle_inactivity_duration
    )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
