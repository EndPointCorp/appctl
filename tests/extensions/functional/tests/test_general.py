"""
General Portal selenium tests.

"""

import os
import time
import tempfile
import subprocess
from functools import partial

import dbus
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.common.by import By
from selenium.webdriver.support import expected_conditions as exp_cond

from base import TestBase
import helpers


class TestSearch(TestBase):
    """
    Test class related to the search box, search button.
    Various interaction scenarios.

    """

    def setup_method(self, method):
        super(TestSearch, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    @helpers.screenshot_on_error
    def test_search_hitting_return_on_search_box(self):
        """
        Test that camera coordinates were changed from the initial
        position to something different after performing the search.

        Interacts with search box and hits the return key on on the
        search box.

        """
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browser)
        pose_start = self.get_camera_pose()
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        # doesn't work if it's not on the correct element ...
        box.send_keys(Keys.RETURN)
        tester = lambda _: self.pose_is_near(pose_start,
                                             self.get_camera_pose(),
                                             assert_alt=False)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until_not(tester, message=msg)

    @helpers.screenshot_on_error
    def test_search_hitting_return_on_search_button(self):
        """
        Test that camera coordinates were changed from the initial
        position to something different after performing the search.

        Interacts with search box and sends return key event on the
        search button.

        """
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browser)
        pose_start = self.get_camera_pose()
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        button = self.browser.find_element_by_class_name("searchbutton")
        button.send_keys(Keys.RETURN)
        # just click() should work according to documentation but doesn't
        # (map doesn't change).
        # action = webdriver.ActionChains(self.browser)
        # action.move_to_element(button).click().perform()
        # says the element is not clickable, confused with size or something
        # the buttons are increased in size by CSS due to the display extension
        # without the extension loaded, even plain this works fine: button.click()
        tester = lambda _: self.pose_is_near(pose_start,
                                             self.get_camera_pose(),
                                             assert_alt=False)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until_not(tester, message=msg)

    @helpers.screenshot_on_error
    def test_search_clicking_search_button(self):
        """
        Test that camera coordinates were changed from the initial
        position to something different after performing the search.

        Interacts with search box and clicks on the search button.

        """
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browser)
        pose_start = self.get_camera_pose()
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        self.click("searchbutton", finder="by_class")
        tester = lambda _: self.pose_is_near(pose_start,
                                             self.get_camera_pose(),
                                             assert_alt=False)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until_not(tester, message=msg)

    @helpers.screenshot_on_error
    def test_no_searchbox_on_other_planets(self):
        """
        The test loads the initial config["maps_url"] and zooms out.
        After this zoom out, the universe objects appear (Earth, Moon, Mars).

        The search box should not be visible when Moon, Mars are clicked.

        First Moon is clicked, disappearance of search box is verified.
        Second, the Earth is clicked, verify search box appeared.
        Last, Mars is clicked, disappearance of search box is verified.

        """
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="acme-poi-button")
        self.click_zoom_out()
        # planets should appear, wait
        # "widget-runway-card-button" does not appear if the zoom out button
        # is not clicked
        tester = partial(exp_cond.visibility_of_element_located,
                         (By.CLASS_NAME, "widget-runway-card-button"))
        msg = "Waiting for element (class: '%s') to appear timed out." % "widget-runway-card-button"
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until(tester(), message=msg)

        # without this additional delay, clicking the planet just
        # doesn't seem to have effect (like if element is not fully loaded in DOM ...?)
        # not sure what to hook the condition to since the Mars, Moon labeled elements
        # are there but clicking them does nothing (without this delay)
        time.sleep(5)

        planets_container = self.browser.find_element_by_id("acme-points-of-interest")
        planets = planets_container.find_elements_by_class_name("widget-runway-card-button")

        # we should have exactly three items in the planets list
        assert len(planets) == 3

        # planets: 0: Earth, 1: Moon, 2: Mars
        planets[1].click()
        # although planets[i].text shows the correct text, clicking the element
        # does not happen if the above additional delay is not there
        # no difference using actions events
        tester = partial(exp_cond.visibility_of_element_located,
                         (By.ID, "searchbox"))
        msg = "Waiting for searchbox to disappear (click on Moon) timed out."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until_not(tester(), message=msg)
        box = self.browser.find_element_by_id("searchbox")
        assert box.is_displayed() is False
        # click on Earth now to make it appear
        # without this delay, the following click won't happen and the
        # explicit wait times out
        time.sleep(5)
        planets[0].click()
        msg = "Waiting for searchbox to appear (click on Earth) timed out."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until(tester(), message=msg)
        box = self.browser.find_element_by_id("searchbox")
        assert box.is_displayed() is True
        # click on Mars now to make it disappear
        # without this delay, the following click won't happen and the
        # explicit wait times out
        time.sleep(5)
        planets[2].click()
        msg = "Waiting for searchbox to disappear (click on Mars) timed out."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until_not(tester(), message=msg)
        box = self.browser.find_element_by_id("searchbox")
        assert box.is_displayed() is False


class TestMiscellaneous(TestBase):
    """
    Other test cases not fitting any other current category.

    """

    def setup_method(self, method):
        super(TestMiscellaneous, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    @helpers.screenshot_on_error
    def test_eu_cookies_info_bar_is_hidden(self):
        """
        Test that the EU cookies info bar is invisible.

        And the map canvas has set top=0px,
        because the cookie container is 30 px higher,
        and the canvas is moved 30 px down.

        """
        # related info:
        # https://redmine.endpoint.com/issues/2517
        # Related patch:
        # https://github.com/EndPointCorp/portal/commit/f7c89fecedd5bcaa94b03289b01393d8cfd9d692
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browser)
        # info bar is of class "pushdown", should be hidden
        bar = self.browser.find_element_by_id("pushdown")
        assert bar.is_displayed() is False
        # NB:
        # when info bar is displayed, both the info bar as well
        # as the widget-scene-canvas (class) have the same location:
        # {'y': 0, 'x': 0}
        # "content-container" (ID) is however shifted when the info
        # bar is present by 30 px, this is should not be now
        content = self.browser.find_element_by_id("content-container")
        assert content.location['x'] == 0
        assert content.location['y'] == 0


class TestOnboard(TestBase):
    """
    Test scenarios covering interaction with the onboard keyboard.
    Onboard is an external system application with which the kiosk
    browser communicates through the onboard chrome extension.

    The tests require ROS running (roslaunch) for communication,
    in particular the onboard ROS node.

    """
    def setup_method(self, method):
        super(TestOnboard, self).setup_method(method)
        # need to start onboard application for the test, handle handle here
        self.onboard_app_output = dict(stdout=tempfile.TemporaryFile("w+"),
                                       stderr=tempfile.TemporaryFile("w+"))
        self.onboard_proc = subprocess.Popen(["/usr/bin/onboard"], **self.onboard_app_output)
        time.sleep(2)
        print "onboard app running, PID: %s" % self.onboard_proc.pid
        assert self.onboard_proc.poll() == None   # if the process runs
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    def teardown_method(self, _):
        # have no doscrings so that the method doesn't appear in the sphinx
        # generated documentation
        # Cleans up running onboard process. Conducted even if the test fails.
        print "onboard app shutdown PID: %s ..." % self.onboard_proc.pid
        self.onboard_proc.kill()
        for _ in range(10):
            if self.onboard_proc.poll() != None:
                break
        print "onboard app return code: %s" % self.onboard_proc.returncode
        map(lambda log: log.seek(0), self.onboard_app_output.values())
        print "onboard app output:\n%s\n%s" % (self.onboard_app_output["stdout"].read(),
                                               self.onboard_app_output["stderr"].read())
        # terminates the browser
        super(TestOnboard, self).teardown_method(_)

    @helpers.screenshot_on_error
    def test_onboard_visibility(self):
        """
        Check whether onboard keyboard shows up after clicking (focusing)
        kiosk searchbox.
        Check whether onboard keyboard disappears after clicking somewhere
        on the canvas (outside the search box).
        Test assertions are made through polling of the onboard app itself
        via dbus python library.

        Related discussion: #134

        """
        # get kiosk browser
        helpers.wait_for_loaded_page(self.config["maps_url"], self.browser)
        bus = dbus.SessionBus()
        remote_object = bus.get_object("org.onboard.Onboard", "/org/onboard/Onboard/Keyboard")
        proxy = dbus.Interface(remote_object, "org.freedesktop.DBus.Properties")
        # use proxy.Get('org.onboard.Onboard.Keyboard', "Visible")
        # which returns Boolean object, something like dbus.Boolean(False, variant_level=1)
        # check onboard keyboard invisible at start
        assert not proxy.Get("org.onboard.Onboard.Keyboard", "Visible")

        action = webdriver.ActionChains(self.browser)
        # put 5 here and it fails on second iteration, details in comments below
        # it's a selenium element handling thing rather than onboard ROS failure
        # manually, it works perfectly fine
        for _ in range(1):
            # search box click, bring it up
            # no other focus interaction works for the search box
            self.click("searchboxinput", finder="by_id")
            time.sleep(2)
            assert proxy.Get('org.onboard.Onboard.Keyboard', "Visible")
            # outside of search box click - click on compass
            # self.click("widget-compass", finder="by_class")
            # doens't work, looks like it's not clicked, yet selenium keeps silent
            compass = self.browser.find_element_by_class_name("widget-compass")
            # with tilt it fails the same, instead of second hide, zoom in happens
            # and the keyboard remains shown
            #tilt = self.browser.find_element_by_class_name("widget-tilt-button-icon")
            action.move_to_element(compass).click().perform()
            # doesn't work at all: self.click("widget-compass", finder="by_class")
            # this is supposed to hind onboard keyboard
            # it gets, in these iterations, hidden just *once*, the second
            # time the keyboard remains shown and the test fails
            # experiments to use canvas, canvas with move_to_element_with_offset
            # did not work
            time.sleep(2)
            assert not proxy.Get("org.onboard.Onboard.Keyboard", "Visible")


class TestFamousPlacesEarthTours(TestBase):
    """
    Famous Places - Earth Tours testing scenarios.

    """
    def setup_method(self, method):
        super(TestFamousPlacesEarthTours, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    def teardown_method(self, _):
        # terminates the browser
        super(TestFamousPlacesEarthTours, self).teardown_method(_)

    def _play_rosbag(self):
        # With a proper ros shell environment (ROS_HOSTNAME, ROS_MASTER_URI, etc), use:
        # $ rosbag play <recorded_file>.bag
        # The bag is just a quick press against some of the SpaceNavigator axes.
        # returns process reference
        bag_file = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                                "data",
                                "spacenav_twist_2015-01-21.bag")
        print "bagfile: '%s'" % bag_file
        print "ROS_HOSTNAME: '%s'" % os.getenv("ROS_HOSTNAME")
        print "ROS_MASTER_URI '%s'" % os.getenv("ROS_MASTER_URI")
        print "PYTHONPATH '%s'" % os.getenv("PYTHONPATH")
        print "LD_LIBRARY_PATH '%s'" % os.getenv("LD_LIBRARY_PATH")
        output = dict(stdout=tempfile.TemporaryFile("w+"),
                      stderr=tempfile.TemporaryFile("w+"))
        proc = subprocess.Popen(["/opt/ros/indigo/bin/rosbag",
                                 "play",
                                 bag_file],
                                **output)
        time.sleep(2)
        print "rosbag app running, PID: %s" % proc.pid
        #print "rosbag poll: ", proc.poll()
        assert proc.poll() == None   # if the process runs
        return proc, output

    @helpers.screenshot_on_error
    def test_earth_tour_interrupted_by_spacenav_activity(self):
        """
        Test Earth tours playback.

        While playing tours emulate spacenav activity by means of playing
        rosbag pre-recorded rosbag file. It's been reported that spacenav
        activity during Earth tour playing resets the tour playback to the
        beginning. Assert that the tour playback doesn't get reset upon
        spacenav activity.

        Emulation of spacenav activity is handled as a background process.

        Scenario: during multiple tours a background process emulates
        the spacenav activity and assertion are made on text boxes with
        time elapsed, so that it doesn't go back to the beginning.

        Discussion / issue ticket:
            #203

        """
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="acme-famous-places-button")
        fp = self.browser.find_element_by_id("acme-famous-places-button")
        assert fp.is_displayed() is True
        fp.click()
        # wait explicitly for tour options to appear

        def tester_tours_inner(browser_ref):
            try:
                tours = self.browser.find_elements_by_class_name("widget-runway-card-button")
                return True if len(tours) > 0 else False
            except:
                return False

        msg = "Tours did not appear within the timeout."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until(partial(tester_tours_inner), message=msg)

        def get_tour_time():
            # remains always empty:
            # elapsed = self.browser.find_element_by_class_name("widget-play-elapsed")
            # duration = self.browser.find_element_by_class_name("widget-play-duration")
            # play_time = self.browser.find_element_by_class_name("widget-play-time")
            # returns elapsed time, total tour duration time
            play_time = self.browser.find_element_by_class_name("widget-play-time")
            print "playtime: '%s'" % play_time.text
            # the text of the element is time info in the form of
            # elapsed time / tour duration (e.g. 00:01/00:34)
            # ignoring the number of minutes, just seconds ...
            elapsed = int(play_time.text.split('/')[0].split(':')[1])
            duration = int(play_time.text.split('/')[1].split(':')[1])
            return elapsed, duration

        # iterate over several particular tours
        # retrieving tours dynamically hits many problems, giving up after several
        # hours of bending things, hitting mainly these problems:
        # - elements in the runway may not be visible, selenium fails
        # - some runway elements are photos (it's difficult to detect a photo if
        #       the label box contains text from a previous video tour)
        # - selenium complains here and there that elements are stale
        # these tours are videos and should be visible:
        tour_labels = ["Arch of Constantine",
                       "Salcantay",
                       "Mt. Fuji",
                       "Japan",
                       "Moab"]
        # play first 'play_duration' seconds of them or the whole of them if shorter
        # some tour items could be pictures which are not considered
        play_duration = 20
        for tour_label in tour_labels:
            # if it's done outside of this playing sub-scenario, next
            # access to the tour.text ends up with:
            # stale element reference: element is not attached to the page document
            tours = self.browser.find_elements_by_class_name("widget-runway-card-button")
            assert len(tours) > 0
            print "total tours: %s" % len(tours)
            for tour in tours:
                if tour.text == tour_label:
                    break
            print "Earth tour played: %s" % tour.text
            tour.click()
            # wait for the tour to properly take off
            # did try checking of the time box or of the upper info box,
            # but they both can be detected from the previous tour
            time.sleep(5)
            # now played ... check if the tour really started:
            for _ in range(self.config["max_load_timeout"]):
                time.sleep(1)
                elapsed, _ = get_tour_time()
                if elapsed > 1:
                    # problem: could be from the previous tour
                    print "continuing (elapsed_sec: %s)" % elapsed
                    print "tour started, is played, waiting ..."
                    break
            # start the process emulating spacenav activity
            rosbag_proc, rosbag_output = self._play_rosbag()
            # wait until the tour playback is finished if shorter than 40s or
            # play only first play_duration seconds
            elapsed_old = elapsed
            for w in range(play_duration - elapsed):
                time.sleep(1)
                elapsed, duration = get_tour_time()
                if elapsed == duration:
                    print "finished before %ss, continuing ..." % play_duration
                    break
                # assert that elapsed time continues
                assert elapsed_old <= elapsed
                elapsed_old = elapsed
            else:
                print "still being played, %ss elapsed, continuing ..." % play_duration
            print "checking the rosbag process ..."
            print "poll(): ", rosbag_proc.poll()
            print "rosbag returncode: ", rosbag_proc.returncode
            map(lambda log: log.seek(0), rosbag_output.values())
            print "output rosbag:\n%s\n%s" % (rosbag_output["stdout"].read(),
                                              rosbag_output["stderr"].read())
            print "tour '%s' done\n%s" % (tour_label, 70 * '-')