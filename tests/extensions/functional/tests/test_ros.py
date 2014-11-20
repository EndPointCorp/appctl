"""
Test involving ROS communication between browsers.

"""

import time
from multiprocessing import Process, Array

import pytest
import rospy
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait

from base import TestBase
from base import TestBaseTouchscreen
from base import MAPS_URL
from base import screenshot_on_error
import helpers
from portal_nav.msg import PortalPose
from base import Pose


class TestBaseSingleBrowserROS(TestBase):
    """
    Tests involving single browser.
    Listening on ROS traffic and asserting on it.

    """

    # extensions to load in the browser
    extensions = ["kiosk"]
    # helper background processes tracking
    processes = []

    def teardown_method(self, _):
        """
        Cleans up running processes. Conducted even if the test fails.

        """
        for p in self.processes:
            if p.is_alive():
                p.terminate()
        # terminates the browser
        super(TestBaseSingleBrowserROS, self).teardown_method(_)

    @staticmethod
    def subscriber(status):
        # msg.current_pose.position.x, msg.current_pose.position.y, msg.current_pose.position.y
        def callback(msg):
            #rospy.loginfo(rospy.get_caller_id() + " received msg: '%s'" % msg)
            # need to make assertions / or rather assumptions here on the
            # received messages but can't pytest.fail from this context
            #print msg.current_pose.position.x, msg.current_pose.position.y, msg.current_pose.position.z
            # can't pass the information to the subscriber() method
            # do the checks here, and last one has either pass or fail
            # the final flag will be set
            if (int(msg.current_pose.position.x) == 16 and
                int(msg.current_pose.position.y) == 49 and
                int(msg.current_pose.position.z) == 18925):
                status.value = "OK"
            else:
                status.value = "ERROR"
            #print status.value

        rospy.init_node("test_subscriber", anonymous=True)
        rospy.Subscriber("/portal_kiosk/current_pose", PortalPose, callback)
        # spin() simply keeps python from exiting until this node is stopped
        print "subscriber listening ..."
        rospy.spin()
        print "subscriber terminated."
        print "final result value: '%s'" % status.value

    @screenshot_on_error
    def test_ros_position_after_search(self):
        """
        Run browser and type something in the search box.
        Helper processes subscriber follows ROS traffic
        and asserts according to the known final values,
        communicates back to this test case the status message.

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
        # delay necessary, otherwise only first messages are caught
        # should be made deterministically though
        time.sleep(5)
        subs.terminate()
        assert status.value == "OK"


class TestBaseTwoBrowsersROS(TestBase):
    """
    Tests involving two browsers.
    General idea is to selenium-manipulate one browser and assert
    accordingly adjusted state (as propagated through ROS) in the
    other browser.

    """

    # extensions to load in browsers
    extensions_browser_1 = ["kiosk"]
    extensions_browser_2 = ["display"]

    def setup_method(self, method):
        self.browser_1 = self.run_browser(self.extensions_browser_1)
        self.browser_2 = self.run_browser(self.extensions_browser_2)
        self.current_method = method.__name__

    def teardown_method(self, _):
        self.browser_1.quit()
        self.browser_2.quit()

    def get_camera_pose(self, browser):
        """
        TODO:
            having browser attribute as part of the test case doesn't seem
            to be ideal, they should be separated.
            Most common operations like this one should be separated from
            test case. Perhaps into helpers for starters.
            Move this method accordingly.
            Wait until
                topic/selenium_tests branch is wiped
                topic/selenium is merged with topic/ros_selenium

        """
        res = browser.execute_script('return acme.getCameraPose();')
        # today is not 'vg', it's 'yg' ...
        return Pose(res['alt'], res['g'], res['yg'])

    def test_ros_positions_in_browsers_aligned_after_kiosk_search(self):
        """
        Start 2 browsers.
        Both need to load MAPS_URL to make acme stuff available.
        Search something in kiosk one, the display one has to adjust.
        Assert on final state.

        TODO:
            blunt time.sleep() not nice

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(MAPS_URL, self.browser_1)
        # browser_2 remains blank ... acme is not defined when trying get_camera_pose, so ...
        # loading the right URL makes acme available
        self.browser_2.get(MAPS_URL)
        #  helpers.wait_for_loaded_page is hooked to compass element which
        #  is not there in display extension ... so just blunt wait
        time.sleep(5)

        pose_start = self.get_camera_pose(self.browser_1)
        box = self.browser_1.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        box.send_keys(Keys.RETURN)
        tester = lambda _: self.pose_is_near(pose_start,
                                             self.get_camera_pose(self.browser_1),
                                             assert_alt=False)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser_1,
                      config["max_load_timeout"]).until_not(tester, message=msg)
        # need to wait a bit for the 'display' browser to finish adjusting itself
        time.sleep(5)
        assert self.pose_is_near(self.get_camera_pose(self.browser_1),
                                 self.get_camera_pose(self.browser_2))