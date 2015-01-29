"""
Tests involving ROS communication between browsers.
Various kiosk-display synchronization tests (action initiated
    in kiosk gets accordingly propagated to display browser).

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
from functools import partial
from multiprocessing import Process, Array

import rospy
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.common.by import By

import helpers
from base import TestBase
from portal_nav.msg import PortalPose
from base import Pose
from test_runway import TestRunway


class TestBaseSingleBrowserROS(TestBase):
    """
    Tests involving single browser.

    Listening and asserting ROS traffic based on the actions performed in
    the (kiosk extension) browser. Browser runs in one process, another
    process is ROS topic listener.

    """

    def setup_method(self, method):
        super(TestBaseSingleBrowserROS, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])
        # helper background processes tracking
        self.processes = []

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
        self.browsers = dict(kiosk=self.run_browser(self.config["chromes"]["kiosk"]),
                             display=self.run_browser(self.config["chromes"]["display"]))
        # helper background processes tracking
        self.processes = []

    def teardown_method(self, _):
        [browser.quit() for browser in self.browsers.values()]
        for p in self.processes:
            if p.is_alive():
                p.terminate()

    @staticmethod
    def subscriber(test_name):
        def callback(msg):
            print "%s ROS message: x: %8.3f   y: %8.3f   z: %15.3f" % (test_name,
                                                                       msg.current_pose.position.x,
                                                                       msg.current_pose.position.y,
                                                                       msg.current_pose.position.z)

        rospy.init_node("test_subscriber", anonymous=True)
        rospy.Subscriber("/portal_kiosk/current_pose", PortalPose, callback)
        # spin() simply keeps python from exiting until this node is stopped
        print "%s debug subscriber listening ..." % test_name
        rospy.spin()
        print "%s debug subscriber terminated." % test_name

    @helpers.screenshot_on_error
    def test_ros_positions_aligned_after_search(self):
        """
        Perform search in the kiosk browser and assert on the automatically
        synchronized final position in the display browser.

        """
        test_name = "test_ros_positions_aligned_after_search"
        subs = Process(target=self.subscriber, args=(test_name,))
        subs.start()
        self.processes.append(subs)

        # display extension browser which has HTML elements displayed,
        # continue when widget-mylocation-button disappears BUT at that point,
        # the page is still not yet fully loaded
        # to make sure the display browser is connected to rosridge properly,
        # start it first and then start kiosk browser
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browsers["display"],
                                     elem_identifier_kind=By.CLASS_NAME,
                                     elem_identifier_name="widget-mylocation-button",
                                     elem_presence=False)
        # make sure that display browser is properly connected
        time.sleep(10)

        # both browsers need to load config["maps_url"] to make acme stuff available,
        # otherwise browser remains blank
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browsers["kiosk"])

        box = self.browsers["kiosk"].find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        box.send_keys(Keys.RETURN)
        babice_pose = Pose(alt=18925, lon=16, lat=49)
        # wait for the 'kiosk' browser to finish adjusting itself
        # unused argument is selenium webdriver (browser) reference
        tester = lambda _: self.pose_is_near(babice_pose,
                                             self.get_camera_pose(self.browsers["kiosk"]),
                                             alt_delta=babice_pose.alt * 0.1,
                                             lon_delta=babice_pose.lon * 0.1,
                                             lat_delta=babice_pose.lat * 0.1)
        msg = "Waiting for position change in the kiosk browser timed out."
        WebDriverWait(self.browsers["kiosk"],
                      self.config["max_load_timeout"]).until(tester, message=msg)
        # wait for the 'display' browser to finish adjusting itself
        # unused argument is selenium webdriver (browser) reference
        tester = lambda _: self.pose_is_near(babice_pose,
                                             self.get_camera_pose(self.browsers["display"]),
                                             alt_delta=babice_pose.alt * 0.1,
                                             lon_delta=babice_pose.lon * 0.1,
                                             lat_delta=babice_pose.lat * 0.1)
        msg = "Waiting for position change in the display browser timed out."
        try:
            WebDriverWait(self.browsers["display"],
                          self.config["max_load_timeout"]).until(tester, message=msg)
        except Exception as ex:
            print "Expected approximate pose values: %s" % (babice_pose, )
            print "Browser kiosk pose values:        %s" % (self.get_camera_pose(self.browsers["kiosk"]), )
            print "Browser display pose values:      %s" % (self.get_camera_pose(self.browsers["display"]), )
            raise

        print "Expected approximate pose values: %s" % (babice_pose, )
        print "Browser kiosk pose values:        %s" % (self.get_camera_pose(self.browsers["kiosk"]), )
        print "Browser display pose values:      %s" % (self.get_camera_pose(self.browsers["display"]), )

        # now wait for both browsers to be (almost) in the same position
        # this final check is a little redundant
        for _ in range(20):
            time.sleep(1)
            kiosk_pose = self.get_camera_pose(self.browsers["kiosk"])
            display_pose = self.get_camera_pose(self.browsers["display"])
            if self.pose_is_near(kiosk_pose, display_pose):
                break
        else:
            # should fail
            assert self.pose_is_near(kiosk_pose, display_pose)

        subs.terminate()

    @helpers.screenshot_on_error
    def test_ros_zoom_synchronization_high_altitude(self):
        """
        Two browsers, kiosk-display, test case.
        Test the display browser gets in sync after zoom in/out action in kiosk.
        Discussion ticket: #139

        """
        test_name = "test_ros_zoom_synchronization_high_altitude"
        subs = Process(target=self.subscriber, args=(test_name,))
        subs.start()
        self.processes.append(subs)

        # display extension browser which has HTML elements displayed,
        # continue when widget-mylocation-button disappears BUT at that point,
        # the page is still not yet fully loaded
        # to make sure the display browser is connected to rosridge properly,
        # start it first and then start kiosk browser
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browsers["display"],
                                     elem_identifier_kind=By.CLASS_NAME,
                                     elem_identifier_name="widget-mylocation-button",
                                     elem_presence=False)
        # both browsers need to load config["maps_url"] to make acme stuff available,
        # otherwise browser remains blank
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browsers["kiosk"])

        def tester(stage):
            time.sleep(5)
            print stage
            kiosk_pose = self.get_camera_pose(self.browsers["kiosk"])
            display_pose = self.get_camera_pose(self.browsers["display"])
            print "Browser kiosk pose values:        %s" % (kiosk_pose, )
            print "Browser display pose values:      %s" % (display_pose, )
            assert self.pose_is_near(kiosk_pose, display_pose)

        tester("INITIAL POSITION")
        self.click_zoom_in(browser=self.browsers["kiosk"])
        tester("AFTER ZOOM IN")
        self.click_zoom_out(browser=self.browsers["kiosk"])
        tester("AFTER ZOOM OUT")
        subs.terminate()

    @helpers.screenshot_on_error
    def test_ros_zoom_synchronization_low_altitude(self):
        """
        Two browsers, kiosk-display, test case.
        Test the display browser gets in sync after zoom in/out action in kiosk.
        Discussion ticket: #139
        This case at very low altitude:
        https://github.com/EndPointCorp/portal/issues/139#issuecomment-70489532
        as explained at this comment, it may be different.
        Non problematic test, works indeed as explained in the comment.

        """
        # display extension browser which has HTML elements displayed,
        # continue when widget-mylocation-button disappears BUT at that point,
        # the page is still not yet fully loaded
        # to make sure the display browser is connected to rosbridge properly,
        # start it first and then start kiosk browser
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browsers["display"],
                                     elem_identifier_kind=By.CLASS_NAME,
                                     elem_identifier_name="widget-mylocation-button",
                                     elem_presence=False)
        # both browsers need to load config["maps_url"] to make acme stuff available,
        # otherwise browser remains blank
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browsers["kiosk"])
        for _ in range(10):
            self.click_zoom_in(browser=self.browsers["kiosk"])
        time.sleep(5)

        def tester(stage):
            time.sleep(5)
            #print stage
            kiosk_pose = self.get_camera_pose(self.browsers["kiosk"])
            display_pose = self.get_camera_pose(self.browsers["display"])
            # print "Browser kiosk pose values:        %s" % (kiosk_pose, )
            # print "Browser display pose values:      %s" % (display_pose, )
            assert self.pose_is_near(kiosk_pose, display_pose)

        tester("INITIAL POSITION")
        self.click_zoom_in(browser=self.browsers["kiosk"])
        tester("AFTER ZOOM IN")
        self.click_zoom_out(browser=self.browsers["kiosk"])
        tester("AFTER ZOOM OUT")

    def _prepare_planets_test(self):
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browsers["display"],
                                     elem_identifier_kind=By.CLASS_NAME,
                                     elem_identifier_name="widget-mylocation-button",
                                     elem_presence=False)
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browsers["kiosk"])
        # maximal zoom out: zoom out (-) button is disabled
        self.click_zoom_out(browser=self.browsers["kiosk"])
        # such a simple click like with acme-poi-button is not possible with
        # the zoom button, gives: unknown error: Element is not clickable at point (423, 420) ...
        zoom_out_button = self.browsers["kiosk"].find_element_by_class_name("widget-zoom-out")
        time.sleep(5)
        if zoom_out_button.is_enabled():
            self.click_zoom_out(browser=self.browsers["kiosk"])
            time.sleep(5)
        assert zoom_out_button.is_enabled() is False
        # now select Points of Interest and check the planets are shown
        poi = self.browsers["kiosk"].find_element_by_id("acme-poi-button")
        poi.click()
        # The first 3 positions are set in POIs tray with planets icons.
        # also it takes some time until the planets appear, wait for the
        # first one in explicit wait ...

        def tester(browser):
            planets_container = browser.find_element_by_id("acme-points-of-interest")
            planets = planets_container.find_elements_by_class_name("widget-runway-card-button")
            # sometimes the container is empty, wait for cup full
            if len(planets) < 3:
                return False
            for p in range(0, 3):
                if planets[p].text not in ("Earth", "Moon", "Mars"):
                    return False
            else:
                return True

        msg = "Planets did not appear within the timeout."
        my_test = partial(tester)
        WebDriverWait(self.browsers["kiosk"],
                      self.config["max_load_timeout"]).until(my_test, message=msg)

    @helpers.screenshot_on_error
    def test_ros_planets_switching(self):
        """
        Zoom out on Earth first so that the icons for Earth, Moon, Mars appear.
        Then iterate over these runway icons and check if the display
        browser reacted accordingly and got in sync with kiosk.
        Discussion ticket: #139

        """
        self._prepare_planets_test()
        planets_container = self.browsers["kiosk"].find_element_by_id("acme-points-of-interest")
        planets = planets_container.find_elements_by_class_name("widget-runway-card-button")
        zoom_out_button = self.browsers["kiosk"].find_element_by_class_name("widget-zoom-out")
        poi = self.browsers["kiosk"].find_element_by_id("acme-poi-button")
        poi.click()
        assert len(planets) == 3

        for i in range(0, 3):
            try:
                # test selecting planets, zoom stays maximal, so assuming that
                # zoom out button remains disabled
                planets[i].click()
                time.sleep(5)
                assert zoom_out_button.is_enabled() is False
                # displayed planet detection:
                #   pose is in a high altitude so it's currently problematic due
                #   to side behaviour of tactile
                # use URL for checking the planet:
                #   https://www.google.com/maps/space/mars/ ...
                #   https://www.google.com/maps/space/moon/ ...
                #   https://www.google.com/maps/@ ...
                # ideally could use of some public API like acme.getPlanet() once provided
                if planets[i].text == "Earth":
                    self.browsers["display"].current_url.startswith("https://www.google.com/maps/@")
                else:
                    # check that URL contains the name of the correct planet
                    assert self.browsers["display"].current_url.find(planets[i].text.lower()) > 0
                # another source of assertion could be a subscriber following
                # communication on the /portal_kiosk/runway topic
            except AssertionError:
                print "AssertionError, len(planets): %s" % len(planets)
                for ii in range(len(planets)):
                    print "%s .. '%s'" % (ii, planets[ii].text)
                    raise

    @helpers.screenshot_on_error
    def test_ros_planets_click_home_icon_subsequent_out_of_sync(self):
        """
        Synchronization break test / verification.

        Being on the Moon and clicking the Home-Earth icon (once is enough)
        breaks position synchronization.

        Once on the Moon, going to altitude low enough so that both kiosk
        and display should have reliably same pose object, assert on the same
        pose object. Once the broken synchronization is fixed, display pose
        object will be in sync with kiosk, this is the final assert here.

        Discussion tickets:
            #111, #139, #297 - basically the same thing ..., most briefly and
                most recently summarized on #297

        """
        self._prepare_planets_test()
        poi = self.browsers["kiosk"].find_element_by_id("acme-poi-button")
        poi.click()
        planets_container = self.browsers["kiosk"].find_element_by_id("acme-points-of-interest")
        planets = planets_container.find_elements_by_class_name("widget-runway-card-button")
        zoom_out_button = self.browsers["kiosk"].find_element_by_class_name("widget-zoom-out")
        assert len(planets) == 3
        # for some bizarre reason, clicking right away [1] which is Moon doesn't do
        # anything, not even kiosk reacts (something to do with selenium, probably),
        # so go Earth first then Moon which works fine
        planets[0].click()  # Earth
        time.sleep(5)
        planets[1].click()  # Moon
        time.sleep(5)
        # should be on Moon now, verify
        assert planets[1].text == "Moon"
        assert self.browsers["display"].current_url.find(planets[1].text.lower()) > 0
        # get the home button handler and click, this causes sync breakage
        home_earth = self.browsers["kiosk"].find_element_by_class_name("acme-zoom-out-earth")
        assert home_earth.is_displayed() is True
        home_earth.click()
        # go to very low altitude where we can rely that getCameraPose() is the
        # same on both kiosk, display without any magic artificial inner adjustments
        for _ in range(10):
            self.click_zoom_in(browser=self.browsers["kiosk"])
        time.sleep(5)
        # verify that display is totally off due to broken sync
        kiosk_pose = self.get_camera_pose(self.browsers["kiosk"])
        display_pose = self.get_camera_pose(self.browsers["display"])
        assert self.pose_is_near(kiosk_pose, display_pose)

    @helpers.screenshot_on_error
    def test_ros_points_of_interest_visit_synchronization(self):
        """
        Test Points of Interest are loaded, load one, exit it and
        load another Point of Interest. Assert that during such
        visits the display browser synchronizes accordingly.

        Do not rely on URL in the browser in asserts, sometimes the
        URLs differ in kiosk, display while showing the same content.

        Assert on titles / caption of the content (i.e. that the
        text in kiosk == display).

        Tests checks that entering and leaving POI works
        properly and the displays are in sync during this actions.

        """
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browsers["display"],
                                     elem_identifier_kind=By.CLASS_NAME,
                                     elem_identifier_name="widget-mylocation-button",
                                     elem_presence=False)
        TestRunway.prepare_poi(self.config, self.browsers["kiosk"])
        poi_container = self.browsers["kiosk"].find_element_by_id("acme-points-of-interest")
        points = poi_container.find_elements_by_class_name("widget-runway-card-button")
        # there is a number of POI but it seems that selenium returns just the
        # visible ones (e.g. 4/20)
        assert len(points) > 0
        # array of elements (author of the text, caption)
        # HTML class name: widget-titlecard-attribution-text
        # array of elements, most of the time just 1 item (e.g. Photo)
        # HTML class name: widget-titlecard-type-label
        classes = ("widget-titlecard-attribution-text", "widget-titlecard-type-label")
        c = 0
        for p in points:
            action = webdriver.ActionChains(self.browsers["kiosk"])
            action.move_to_element(p).click().perform()
            time.sleep(5)
            for klass in classes:
                elems_kiosk = self.browsers["kiosk"].find_elements_by_class_name(klass)
                elems_display = self.browsers["display"].find_elements_by_class_name(klass)
                for e_kiosk, e_display in zip(elems_kiosk, elems_display):
                    assert e_kiosk.text == e_display.text
            if c > 5:
                break
            c += 1