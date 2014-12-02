"""
Tests related to Runway elements.

The Runway is the element at to bottom of the touchscreen browser,
it contains a list of Points of Interests
(if the camera is zoomed in to some level, and there are some
interesting places around) or the list consisting of the Earth, Moon, Mars.

There is also a list of Famous Places (Earth tours), which is always
filled (loaded by a static list of entries read from a file).

"""


import time
from functools import partial

import pytest
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait

from base import screenshot_on_error
from base import TestBase
import helpers


class TestRunway(TestBase):
    """
    Runway element interaction tests.

    """

    extensions = ["kiosk"]

    @screenshot_on_error
    def test_runway_buttons_basic(self):
        """
        Test that Point of Interest and Famous Places are displayed.

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(config["maps_url"],
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
        config = self.get_config()
        helpers.wait_for_loaded_page(config["maps_url"],
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
            planets_container = self.browser.find_element_by_id("acme-points-of-interest")
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
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(my_test, message=msg)

        planets_container = self.browser.find_element_by_id("acme-points-of-interest")
        planets = planets_container.find_elements_by_class_name("widget-runway-card-button")

        assert len(planets) == 3

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
        config = self.get_config()
        # beware, with URL copied from chrome, our extensions may not be present
        # better to stick to the tested URLs ... (buttons Points of Interest and
        # Famous Places are not there, though Points of Interest tray is filled)
        helpers.wait_for_loaded_page(config["maps_url"],
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
        poi_button = self.browser.find_element_by_id("acme-poi-button")
        poi_button.click()

        # it takes some time until they appear, wait explicitly
        def tester(browser):
            poi_container = self.browser.find_element_by_id("acme-points-of-interest")
            pois = poi_container.find_elements_by_class_name("widget-runway-card-button")
            # sometimes the first POI has no label (.text), if there are more items
            # check their .text labels
            try:
                for poi in pois:
                    if poi.text != '':
                        return True
                else:
                    return False
            # StaleElementReferenceException: Message: u'stale element reference:
            # element is not attached to the page document
            except:
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
        poi_container = self.browser.find_element_by_id("acme-points-of-interest")
        points = poi_container.find_elements_by_class_name("widget-runway-card-button")
        assert len(points) > 0
        c = 0
        for p in points:
            action = webdriver.ActionChains(self.browser)
            action.move_to_element(p).click().perform()
            time.sleep(2)
            if c > 5:
                break
            c += 1

    @pytest.mark.skipif(True, reason="Unstable camera pose object attributes, reported.")
    @screenshot_on_error
    def test_runway_check_earth_icon_click(self):
        """
        The Earth icon (most left picture), clicking it should bring the
        view to a considerably zoomed out position.

        NB: position object values differ between subsequent runs.

        """
        config = self.get_config()
        # position object is available after the browser loads
        # our special URL, otherwise failing with:
        # WebDriverException: Message: u'unknown error: acme is not defined\n
        helpers.wait_for_loaded_page(config["maps_url"],
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
        assert earth_click_pose.alt > after_search_pose.alt * 100
        # click again now, should how into large altitude,
        # comparable to earth_click_pose
        earth = self.browser.find_element_by_class_name("acme-zoom-out-earth")
        assert earth.is_displayed() is True
        earth.click()
        # 2nd earth icon click
        # tolerate 30% difference from the target altitude value
        # just wait until the position changes, wait for the final
        # asserted condition. waiting for first pose change is not the
        # it been observed that this conditional wait fulfills at some threshold
        # but the position still changes, so we satisfy on 30% difference
        tester = lambda _: self.pose_is_near(earth_click_pose,
                                             self.get_camera_pose(),
                                             alt_delta=earth_click_pose.alt * 0.3,
                                             assert_lon=False,
                                             assert_lat=False)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(tester, message=msg)
        earth = self.browser.find_element_by_class_name("acme-zoom-out-earth")
        assert earth.is_displayed() is True