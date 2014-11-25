"""
Test involving ROS communication between browsers.

"""

import os
import pytest

# this is basic ROS check, there needs to be more to run the ROS
# tests from this module (README.md):
#   catkin workspace setup file sourced
#   roslaunch running
if not os.environ.get("ROS_DISTRO", None):
    msg = "ROS is not configured, skipping ROS tests ..."
    print "\n%s" % msg
    pytest.skip(msg="ROS is not configured, skipping ROS tests ...")
print "\nROS - running tests"

import time
from multiprocessing import Process, Array

import rospy
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait

from base import TestBase
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
        count = 0
        while True:
            if status.value == "OK":
                break
            time.sleep(1)
            count += 1
            if count > 30:
                pytest.fail("Waiting for correct position ROS msg timed out.")
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

    def test_ros_positions_in_browsers_aligned_after_kiosk_search(self):
        """
        Start 2 browsers.
        Both need to load MAPS_URL to make acme stuff available.
        Search something in kiosk one, the display one has to adjust.
        Assert on final state.

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(MAPS_URL, self.browser_1)
        # browser_2 remains blank ... acme is not defined when trying get_camera_pose, so ...
        # loading the right URL makes acme available
        self.browser_2.get(MAPS_URL)
        # TODO
        #  helpers.wait_for_loaded_page is hooked to compass element which
        #  is not there in display extension ... so just blunt wait
        time.sleep(5)

        box = self.browser_1.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        box.send_keys(Keys.RETURN)
        babice_pose = Pose(alt=18925.2298526623, lon=16.69756065, lat=49.28254545)
        tester = lambda _: self.pose_is_near(babice_pose,
                                             self.get_camera_pose(self.browser_1))
        msg = "Waiting for position change in the kiosk browser timed out."
        WebDriverWait(self.browser_1,
                      config["max_load_timeout"]).until(tester, message=msg)
        # wait for the 'display' browser to finish adjusting itself
        tester = lambda _: self.pose_is_near(babice_pose,
                                             self.get_camera_pose(self.browser_2))
        msg = "Waiting for position change in the display browser timed out."
        WebDriverWait(self.browser_2,
                      config["max_load_timeout"]).until(tester, message=msg)
        assert self.pose_is_near(self.get_camera_pose(self.browser_1),
                                 self.get_camera_pose(self.browser_2))