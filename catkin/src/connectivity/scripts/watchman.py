#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import rospy
import urllib2
import os
from appctl.msg import Mode
from connectivity.srv import Online


class HeadRequest(urllib2.Request):
    def get_method(self):
        return "HEAD"


class ConnectivityOverlord():
    def __init__(self,
                 timeout=1,
                 max_failed_attempts=5,
                 offline_mode_name='offline_video',
                 online_mode_name='tactile'):
        """
        By default we make 5 HTTP:HEAD check on google.com + google.com/maps
        with 1 sec timeout and publish a mode change of "offline_video"
        on /appctl/mode
        """

        self.online = True
        self.timeout = timeout
        self.online_mode_name = 'tactile'
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
        for url, failures in self.sites.iteritems():
            if self._got_response(url):
                failures = 0
                rospy.loginfo("%s response - internet is fine" % url)
            else:
                failures += 1
                rospy.loginfo("%s no response - internet is having problems" % url)

        if not self._got_internet() and self.online:
            """ We've just lost internetz """
            self._publish_offline_mode()
            self.online = False
        elif self._got_internet() and self.online:
            """ Internetz came back """
            self._publish_online_mode()
            self.online = True

        pass

    def _publish_offline_mode(self):
        """ Emits a message on /appctl/mode about the mode change """
        offline_msg = Mode(mode=self.offline_mode_name)
        self.mode_publisher.publish(offline_msg)
        pass

    def _publish_online_mode(self):
        """ Emits a message on /appctl/mode about the mode change """
        online_msg = Mode(mode=self.online_mode_name)
        self.mode_publisher.publish(online_msg)
        pass

    def _got_response(self, url):
        try:
            urllib2.urlopen(HeadRequest(url), timeout=self.timeout)
            return True
        except Exception:
            return False

    def _init_session_service(self):
        service = rospy.Service('/connectivity/online',
                                Online,
                                self._process_service_request)
        return service

    def _process_service_request(self, req):
        return self._got_internet()

    def _got_internet(self):
        """
        If any of the values of self.sites is above max_failed_attempts then
        return "False".

        There's a special /tmp/test_offline_mode file that will turn your portal
        to offline mode
        """
        if os.path.isfile('/tmp/test_offline_mode'):
            return False

        return all(x < self.max_failed_attempts for x in self.sites.itervalues())

    def _sleep_between_requests(self):
        rospy.sleep(1)
        pass

    def run(self):
        rospy.logdebug("Starting connectivity service")
        while not rospy.is_shutdown():
            self._sleep_between_requests()
            self.update_internetz_status()
        pass

if __name__ == '__main__':
    rospy.init_node('connectivity')

    timeout = rospy.get_param('~timeout', 1)
    max_failed_attempts = rospy.get_param('~max_failed_attempts', 5)
    offline_mode_name = rospy.get_param('~offline_mode_name', 'offline_video')
    online_mode_name = rospy.get_param('~offline_mode_name', 'tactile')

    try:
        ConnectivityOverlord(timeout=timeout,
                             max_failed_attempts=max_failed_attempts,
                             offline_mode_name=offline_mode_name,
                             online_mode_name=online_mode_name).run()
    except rospy.ROSInterruptException:
        pass
    except rospy.service.ServiceException:
        pass
