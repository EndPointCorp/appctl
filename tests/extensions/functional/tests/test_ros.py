"""
ROS - base tests.

"""

import time
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
import rospy
import pytest
from multiprocessing import Process, Array

from base import TestBase
from base import TestBaseTouchscreen
from base import MAPS_URL
from base import screenshot_on_error
import helpers
from portal_nav.msg import PortalPose


class TestBaseROS(TestBase):
    """

    """

    extensions = ["kiosk"]
    processes = []

    def teardown_method(self, _):
        """
        Cleans up running processes. Conducted even if the test fails.

        """
        for p in self.processes:
            if p.is_alive():
                p.terminate()
        super(TestBaseROS, self).teardown_method(_)

    def subscriber(self, status):
        # msg.current_pose.position.x, msg.current_pose.position.y, msg.current_pose.position.y
        def callback(msg):
            #rospy.loginfo(rospy.get_caller_id() + " received msg: '%s'" % msg)
            # need to make assertions / or rather assumptions here on the
            # received messages but can't pytest.fail from this context
            print msg.current_pose.position.x, msg.current_pose.position.y, msg.current_pose.position.z
            # can't pass the information to the subscriber() method
            # do the checks here, and last one has either pass or fail
            # the final flag will be set
            if (int(msg.current_pose.position.x) == 16 and
                int(msg.current_pose.position.y) == 49 and
                int(msg.current_pose.position.z) == 18925):
                status.value = "OK"
            else:
                status.value = "ERROR"
            print status.value

        rospy.init_node("test_subscriber", anonymous=True)
        rospy.Subscriber("/portal_kiosk/current_pose", PortalPose, callback)
        # spin() simply keeps python from exiting until this node is stopped
        print "subscriber listening ..."
        rospy.spin()
        print "subscriber terminated."
        print "final result value: '%s'" % status.value

    @screenshot_on_error
    def test_ros_basic(self):
        """

        """
        status = Array('c', "ERROR")
        subs = Process(target=self.subscriber, args=(status, ))
        subs.start()
        self.processes.append(subs)
        # start browser now
        helpers.wait_for_loaded_page(MAPS_URL, self.browser)
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        self.click("searchbutton", finder="by_class")
        # delay necessary, otherwise not only first messages are caught
        # should be made deterministically though
        time.sleep(3)
        subs.terminate()
        assert status.value == "OK"