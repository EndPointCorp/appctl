#!/usr/bin/env python

import rospy
import time
from statistics.msg import Session
from std_msgs.msg import Duration
from std_msgs.msg import String
from appctl.msg import Mode
from appctl.srv import Query

DEFAULT_SESSION_TIMEOUT = 20.0 # seconds
FALLBACK_MODE = 'tactile'
OFFLINE_MODE = 'offline_video'
IGNORE_MODES = 'offline_video,attended'
PORTAL_LOADER_URL = 'http://lg-head/portal-loader.html'


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
                 kiosk_switcher_publisher,
                 offline_mode,
                 mode_service,
                 ignore_modes,
                 session_publisher):
        self.inactivity_timeout = inactivity_timeout
        self.session_publisher = session_publisher
        self.mode_service = mode_service
        self.fallback_mode = fallback_mode
        self.fallback_publisher = fallback_publisher
        self.kiosk_switcher_publisher = kiosk_switcher_publisher
        self.offline_mode = offline_mode
        self.ignore_modes = ignore_modes.split(',')
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
        Don't do it if we're in offline mode
        """
        end_msg = Session(end_ts=int(time.time()))
        current_mode = self._call_mode_service()
        if (current_mode != self.offline_mode) and (current_mode not in self.ignore_modes):
            """ Dont fallback to ambient mode if we're in offline mode or e.g. attended mode"""
            fallback_mode = Mode(mode=self.fallback_mode)
            self.fallback_publisher.publish(fallback_mode)
            self.kiosk_switcher_publisher.publish(String(PORTAL_LOADER_URL))
        else:
            rospy.loginfo("Not switching to %s because we're in %s" % (self.fallback_mode, self.offline_mode))
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

    kiosk_switcher_publisher = rospy.Publisher(
        '/kiosk/switch',
        String,
        queue_size=2
    )

    offline_mode = rospy.get_param(
        '~offline_mode',
        OFFLINE_MODE
    )

    ignore_modes = rospy.get_param(
        '~ignore_modes',
        IGNORE_MODES
    )

    mode_service = rospy.ServiceProxy('appctl/query', Query)

    ender = SessionBreaker(inactivity_timeout=inactivity_timeout,
                           fallback_mode=fallback_mode,
                           fallback_publisher=fallback_publisher,
                           kiosk_switcher_publisher=fallback_publisher,
                           offline_mode=offline_mode,
                           session_publisher=session_publisher,
                           ignore_modes=ignore_modes,
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
