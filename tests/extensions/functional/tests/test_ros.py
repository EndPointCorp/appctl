"""
Tests involving ROS communication between browsers.

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
from selenium.webdriver.common.by import By

import helpers
from base import TestBase
from portal_nav.msg import PortalPose
from base import Pose


class TestBaseSingleBrowserROS(TestBase):
    """
    Tests involving single browser.

    Listening and asserting ROS traffic based on the actions performed in
    the (kiosk extension) browser. Browser runs in one process, another
    process is ROS topic listener.

    """

    # helper background processes tracking
    processes = []

    def setup_method(self, method):
        super(TestBaseSingleBrowserROS, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    def teardown_method(self, _):
        # have no doscrings so that the method doesn't appear in the sphinx
        # generated documentation
        # Cleans up running processes. Conducted even if the test fails.
        for p in self.processes:
            if p.is_alive():
                p.terminate()
        # terminates the browser
        super(TestBaseSingleBrowserROS, self).teardown_method(_)

    @staticmethod
    def subscriber(status):
        catches = []
        def callback(msg):
            #rospy.loginfo(rospy.get_caller_id() + " received msg: '%s'" % msg)
            # need to make assertions / or rather assumptions here on the
            # received messages but can't pytest.fail from this context
            catches.append([msg.current_pose.position.x,
                            msg.current_pose.position.y,
                            msg.current_pose.position.z])
            # can't pass the information to the subscriber() method
            # do the checks here, and last one has either pass or fail
            # the final flag will be set
            if (int(msg.current_pose.position.x) in range(14, 17) and
                int(msg.current_pose.position.y) in range(47, 50) and
                # the .z value is 35061 on jenkins
                int(msg.current_pose.position.z) in range(18000, 40000)):
                status.value = "OK"
            else:
                status.value = "ERROR"

        if status.value == "ERROR":
            for catch in catches:
                print catch
        rospy.init_node("test_subscriber", anonymous=True)
        rospy.Subscriber("/portal_kiosk/current_pose", PortalPose, callback)
        # spin() simply keeps python from exiting until this node is stopped
        print "subscriber listening ..."
        rospy.spin()
        print "subscriber terminated."
        print "final result value: '%s'" % status.value

    @helpers.screenshot_on_error
    def test_ros_position_after_search(self):
        """
        Run browser and type something in the search box, a place with
        a known position.

        Helper background process listening on **/portal_kiosk/current_pose**
        ROS topic and checks ROS position messages until the one expected
        arrives (within certain timeout).

        Pass/fail flag is set accordingly by the listener process via
        shared memory value which this test cases evaluates eventually.

        """
        status = Array('c', "ERROR")
        subs = Process(target=self.subscriber, args=(status, ))
        subs.start()
        self.processes.append(subs)
        # start browser now
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browser)
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
            if count > self.config["max_load_timeout"]:
                pytest.fail("Waiting for correct position ROS msg timed out.")
        subs.terminate()
        assert status.value == "OK"


class TestBaseTwoBrowsersROS(TestBase):
    """
    Tests involving two browsers.

    General idea is to selenium-manipulate one browser (with kiosk
    extension) and assert accordingly adjusted state (as propagated
    through ROS) in the other browser (display extension).

    """

    def setup_method(self, method):
        super(TestBaseTwoBrowsersROS, self).setup_method(method)
        self.browser_1 = self.run_browser(self.config["chromes"]["kiosk"])
        self.browser_2 = self.run_browser(self.config["chromes"]["display"])

    def teardown_method(self, _):
        self.browser_1.quit()
        self.browser_2.quit()

    def test_ros_positions_in_browsers_aligned_after_kiosk_search(self):
        """
        Perform search in the kiosk browser and assert on the automatically
        synchronized final position in the display browser.

        """
        # both browsers need to load config["maps_url"] to make acme stuff available,
        # otherwise browser remains blank
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browser_1)

        # browser_2, with display extension which has HTML elements displayed,
        # continue when widget-mylocation-button disappears BUT at that point,
        # the page is still not yet fully loaded
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browser_2,
                                     elem_identifier_kind=By.CLASS_NAME,
                                     elem_identifier_name="widget-mylocation-button",
                                     elem_presence=False)

        box = self.browser_1.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        box.send_keys(Keys.RETURN)
        babice_pose = Pose(alt=18925, lon=16, lat=49)
        # wait for the 'kiosk' browser to finish adjusting itself
        # unused argument is selenium webdriver (browser) reference
        tester = lambda _: self.pose_is_near(babice_pose,
                                             self.get_camera_pose(self.browser_1),
                                             alt_delta=babice_pose.alt * 0.1,
                                             lon_delta=babice_pose.lon * 0.1,
                                             lat_delta=babice_pose.lat * 0.1)
        msg = "Waiting for position change in the kiosk browser timed out."
        WebDriverWait(self.browser_1,
                      self.config["max_load_timeout"]).until(tester, message=msg)
        # wait for the 'display' browser to finish adjusting itself
        # unused argument is selenium webdriver (browser) reference
        tester = lambda _: self.pose_is_near(babice_pose,
                                             self.get_camera_pose(self.browser_2),
                                             alt_delta=babice_pose.alt * 0.1,
                                             lon_delta=babice_pose.lon * 0.1,
                                             lat_delta=babice_pose.lat * 0.1)
        msg = "Waiting for position change in the display browser timed out."
        WebDriverWait(self.browser_2,
                      self.config["max_load_timeout"]).until(tester, message=msg)
        assert self.pose_is_near(self.get_camera_pose(self.browser_1),
                                 self.get_camera_pose(self.browser_2))