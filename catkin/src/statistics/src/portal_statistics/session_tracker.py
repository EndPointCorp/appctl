#!/usr/bin/env python

import rospy
import time
from statistics.msg import Session
from std_msgs.msg import Duration
from appctl.msg import Mode
from appctl.srv import Query

DEFAULT_SESSION_TIMEOUT = 20.0 # seconds
FALLBACK_MODE = 'tactile'

class SessionBreaker:
    """
    - watches the inactivity duration and sends Session:end_ts=0 to statistics aggregator
    to end current session if it exists
    - if occupancy is broken, starts new session in "tactile" mode which is a default fallback session
    - session breaker should check if we're not running in offline mode
    """

    def __init__(self,
                 inactivity_timeout,
                 fallback_mode,
                 fallback_publisher,
                 mode_service,
                 session_publisher):
        self.inactivity_timeout = inactivity_timeout
        self.session_publisher = session_publisher
        self.mode_service = mode_service
        self.fallback_mode = fallback_mode
        self.fallback_publisher = fallback_publisher
        self.ended = False

    def _wait_for_mode_service(self):
        rospy.logdebug("Waiting for the /appctl/query service to become available")
        rospy.wait_for_service(self.mode_service.resolved_name)
        rospy.logdebug("Appctl/query service has become available")
        pass

    def _call_mode_service(self):
        """ Returns string with current mode """
        service_call = self.mode_service
        response = service_call()
        rospy.logdebug("Received response from service: %s" % response)
        return response.mode

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
        """
        Start new session and make the occupancy_triggered = True
        """
        start_msg = Session(
                            start_ts=int(time.time()),
                            mode=self.fallback_mode,
                            occupancy_triggered=1
                            )

        self.session_publisher.publish(start_msg)

    def _offline_mode(self):
        pass


    def publish_session_end(self):
        """ Switch to tactile to ambient_mode and end the sssion.
        Don't do it if we're offline
        """
        end_msg = Session(end_ts=int(time.time()))
        if self._call_mode_service() != 'offline':
            fallback_mode = Mode(mode=self.fallback_mode)
            self.fallback_publisher.publish(fallback_mode)
        self.session_publisher.publish(end_msg)


def main():
    rospy.init_node('session_ender')

    inactivity_timeout_s = rospy.get_param(
        '~inactivity_timeout',
        DEFAULT_SESSION_TIMEOUT
    )

    fallback_mode = rospy.get_param(
        '~fallback_mode',
        FALLBACK_MODE
    )
    inactivity_timeout = rospy.Duration.from_sec(float(inactivity_timeout_s))


    session_publisher = rospy.Publisher(
        '/statistics/session',
        Session,
        queue_size=2
    )

    fallback_publisher = rospy.Publisher(
        '/appctl/mode',
        Mode,
        queue_size=2
    )

    mode_service = rospy.ServiceProxy('appctl/query', Query)

    ender = SessionBreaker(inactivity_timeout=inactivity_timeout,
                           fallback_mode=fallback_mode,
                           fallback_publisher=fallback_publisher,
                           session_publisher=session_publisher,
                           mode_service=mode_service)

    inactivity_sub = rospy.Subscriber(
        '/portal_occupancy/interaction/inactive_duration',
        Duration,
        ender.handle_inactivity_duration
    )

    rospy.spin()


if __name__ == '__main__':
    main()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
