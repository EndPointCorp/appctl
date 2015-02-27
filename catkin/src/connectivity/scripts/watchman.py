#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import urllib2
from std_msgs import Bool
from appctl import Mode


class HeadRequest(urllib2.Request):
    def get_method(self):
        return "HEAD"


class ConnectivityOverlord():
    def __init__(self, max_failed_attempts=5,
                 timeout=1, offline_mode_name='offline_video'):
        """
        By default we make 5 HTTP:HEAD check on google.com + google.com/maps
        with 1 sec timeout and publish a mode change of "offline_video"
        on /appctl/mode
        """

        self.timeout = timeout
        self.offline_mode_name = 'offline_video'
        self.max_failed_attempts = max_failed_attempts
        self.sites = {"https://google.com": 0, "https://google.com/maps": 0}
        self._init_session_service()
        self.mode_publisher = rospy.Publisher(
                '/appctl/mode',
                Mode,
                queue_size=2
                )

    def update_internetz_status(self):
        """
        Make the requests and increment failure counts upon failure
        Reset the failure count upon success
        """
        for url, failures in self.sites:
            if self._got_response(url):
                failures = 0
                rospy.logdebug("%s response - internet is fine" % url)
            else:
                failures += 3
                rospy.loginfo("%s response - internet is fine" % url)

        if not self._got_internet():
            self._publish_offline_mode()

        pass

    def _publish_offline_mode(self):
        """ Emits a message on /appctl/mode about the mode change """
        offline_msg = Mode(mode=self.offline_mode_name)
        self.mode_publisher.publish(offline_msg)
        pass

    def _got_response(self, url):
        try:
            urllib2.urlopen(HeadRequest(url), timeout=self.timeout)
            return True
        except Exception:
            return False

    def _init_session_service(self):
        service = rospy.Service('/connectivity/online',
                                Bool,
                                self._process_service_request)
        return service

    def _process_service_request(self, req):
        return self._got_internet()

    def _got_internet(self):
        """
        If any of the values of self.sites is above max_failed_attempts then
        return "False"
        """
        return all(x > self.max_failed_attempts for x in self.sites.itervalues())

    def run(self):
        rospy.logdebug("Starting connectivity service")
        while not rospy.is_shutdown():
            self._sleep_between_requests()
            self.update_internetz_status()
            self._spin()
            pass
        pass

if __name__ == '__main__':
    try:
        ConnectivityOverlord().run()
    except rospy.ROSInterruptException:
        pass
    except rospy.service.ServiceException:
        pass
