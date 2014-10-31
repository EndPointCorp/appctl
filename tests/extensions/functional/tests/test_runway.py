"""
The runway is the element at to bottom of the touchscreen browser,
it contains a list of Points of Interests
(if the camera is zoomed in to some level, and there are some
interesting places around) or the list of planets to load (Earth, Moon, Mars).

There is also a list of Famous Places, which is always filled,
as we load there a static list of entries read from a file.

Tickets: Redmine #2511, Github: #133

"""


import time
from functools import partial

import pytest
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait

from base import MAPS_URL, ZOOMED_IN_MAPS_URL
from base import screenshot_on_error, make_screenshot
from base import TestBase
import helpers


class TestRunway(TestBase):

    extensions = ["kiosk"]

    @screenshot_on_error
    def test_runway_buttons_basic(self):
        """
        Test Point of Interest and Famous Places are there

        """
        helpers.wait_for_loaded_page(MAPS_URL,
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="acme-poi-button")
        poi = self.browser.find_element_by_id("acme-poi-button")
        assert poi.is_displayed() is True
        fp = self.browser.find_element_by_id("acme-famous-places-button")
        assert fp.is_displayed() is True
        # select Point of Interest / Famous Places can be done
        fp.click()
        poi.click()

    @screenshot_on_error
    def test_runway_planets_on_max_zoom_out(self):
        """
        Test there is Mars, Earth, Moon loaded
        in Points of Interest tray on maximal zoom out.

        """
        helpers.wait_for_loaded_page(MAPS_URL,
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="acme-poi-button")
        # maximal zoom out: zoom out (-) button is disabled
        self.click_zoom_out()
        # such a simple click like with acme-poi-button is not possible with
        # the zoom button, gives: unknown error: Element is not clickable at point (423, 420) ...
        zoom_out_button = self.browser.find_element_by_class_name("widget-zoom-out")
        assert zoom_out_button.is_enabled() is False
        # now select Points of Interest and check the planets are shown
        poi = self.browser.find_element_by_id("acme-poi-button")
        poi.click()
        # The first 3 positions are set in POIs tray with planets icons.
        # also it takes some time until the planets appear, wait for the
        # first one in explicit wait ...
        config = self.get_config()

        def tester(browser):
            tray = browser.find_element_by_class_name("widget-runway-tray-wrapper")
            planets = tray.find_elements_by_class_name("widget-runway-card-button")
            for i in range(0, 3):
                if planets[i].text not in ("Earth", "Moon", "Mars"):
                    return False
            else:
                return True

        msg = "Planets did not appear within the timeout."
        my_test = partial(tester)
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(my_test, message=msg)

        tray = self.browser.find_element_by_class_name("widget-runway-tray-wrapper")
        planets = tray.find_elements_by_class_name("widget-runway-card-button")

        for i in range(0, 3):
            try:
                # test selecting planets, zoom stays maximal
                planets[i].click()
                time.sleep(3)
                assert zoom_out_button.is_enabled() is False
            except AssertionError:
                print "AssertionError, len(planets): %s" % len(planets)
                for ii in range(len(planets)):
                    print "%s .. '%s'" % (ii, planets[ii].text)
                    raise

    def prepare_poi(self):
        """
        Prepare for Points of Interest tests.
        Load browser, search for a location with POIs and wait
        some time for the runway tray to populate.

        """
        # beware, with URL copied from chrome, our extensions may not be present
        # better to stick to the tested URLs ... (buttons Points of Interest and
        # Famous Places are not there, though Points of Interest tray is filled)
        helpers.wait_for_loaded_page(MAPS_URL,
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="acme-poi-button")
        # change somewhere where there are POIs
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice, poland")
        box.send_keys(Keys.RETURN)
        # have to bluntly wait, can't test Pose position object since with
        # just kiosk extension loaded it fails that acme object is not defined
        # another thing: it takes some time until POIs tray is filled even
        # after changing to the target location
        poi = self.browser.find_element_by_id("acme-poi-button")
        poi.click()

        # it takes some time until they appear, wait explicitly
        def tester(browser):
            tray = browser.find_element_by_class_name("widget-runway-tray-wrapper")
            pois = tray.find_elements_by_class_name("widget-runway-card-button")
            if len(pois) > 0 and pois[0].text != '':
                return True
            else:
                return False

        msg = "POIs did not appear within the timeout."
        my_test = partial(tester)
        config = self.get_config()
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(my_test, message=msg)

    @screenshot_on_error
    def test_runway_points_of_interest(self):
        """
        Test Points of Interest is loaded, we can load one, exit it and
        and load another Point of Interest.

        """
        self.prepare_poi()
        tray = self.browser.find_element_by_class_name("widget-runway-tray-wrapper")
        points = tray.find_elements_by_class_name("widget-runway-subview-card-view-container")
        assert len(points) > 0
        c = 0
        for p in points:
            action = webdriver.ActionChains(self.browser)
            action.move_to_element(p).click().perform()
            time.sleep(2)
            if c > 5:
                break
            c += 1

    @screenshot_on_error
    def test_runway_check_earth_icon_click(self):
        """
        The Earth icon (most left picture), clicking it should bring the
        view to a considerably zoomed out position.
        NB: position object value differ between subsequent runs.

        """
        config = self.get_config()
        # position object is available after the browser loads
        # our special URL, otherwise failing with:
        # WebDriverException: Message: u'unknown error: acme is not defined\n
        helpers.wait_for_loaded_page(MAPS_URL,
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="acme-poi-button")
        init_pose = self.get_camera_pose()
        earth = self.browser.find_element_by_class_name("acme-zoom-out-earth")
        assert earth.is_displayed() is True
        earth.click()
        # just wait until the position changes
        tester = lambda _: self.pose_is_near(init_pose,
                                             self.get_camera_pose(),
                                             alt_delta=init_pose.alt * 0.1)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until_not(tester, message=msg)
        earth = self.browser.find_element_by_class_name("acme-zoom-out-earth")
        assert earth.is_displayed() is True
        earth_click_pose = self.get_camera_pose()
        # now search for something, that is very much zoomed in view,
        # this call waits already
        self.prepare_poi()
        # retrieve the object again, getting otherwise:
        # StaleElementReferenceException: Message: u'stale element reference:
        #   element is not attached to the page document
        earth = self.browser.find_element_by_class_name("acme-zoom-out-earth")
        assert earth.is_displayed() is True
        # since the view is zoomed in, the altitude of the before view
        # should still be 1000 greater than now
        after_search_pose = self.get_camera_pose()
        assert earth_click_pose.alt > after_search_pose.alt * 1000
        # click again now, should how into large altitude,
        # comparable to earth_click_pose
        earth = self.browser.find_element_by_class_name("acme-zoom-out-earth")
        assert earth.is_displayed() is True
        earth.click()
        # just wait until the position changes, wait for the final
        # asserted condition. waiting for first pose change is not the
        # final position and the subsequent assertion fails
        tester = lambda _: self.pose_is_near(earth_click_pose,
                                             self.get_camera_pose(),
                                             alt_delta=earth_click_pose.alt * 0.2,
                                             assert_lon=False,
                                             assert_lat=False)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(tester, message=msg)
        earth = self.browser.find_element_by_class_name("acme-zoom-out-earth")
        assert earth.is_displayed() is True
        earth_2nd_click_pose = self.get_camera_pose()
        # tolerate 20% difference from the target altitude value
        assert self.pose_is_near(earth_click_pose,
                                 earth_2nd_click_pose,
                                 alt_delta=earth_2nd_click_pose.alt * 0.2,
                                 assert_lon=False,
                                 assert_lat=False)