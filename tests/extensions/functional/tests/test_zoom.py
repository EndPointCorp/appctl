"""
Zoom buttons tests.

"""

import time
from functools import partial

import pytest
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as exp_cond

from base import TestBaseTouchscreen
from base import MAPS_URL, ZOOMED_IN_MAPS_URL, Pose
from base import screenshot_on_error, make_screenshot


class TestZoomButtons(TestBaseTouchscreen):
    """
    Simple test for checking the zoom buttons.

    """

    @screenshot_on_error
    def test_zoom_buttons(self):
        self.browser.get(MAPS_URL)
        # this is the container for the two zoom buttons
        zoom = self.browser.find_element_by_id('zoom')
        assert zoom.is_displayed() is True
        for z in [zoom.find_element_by_class_name('widget-zoom-in'),
                  zoom.find_element_by_class_name('widget-zoom-out')]:
            assert z.is_displayed() is True

    def _zoom_clicks_init(self):
        """
        Load the initial URL and wait for loading the page.
        Loading finished conditions identified by 'widget-compass' on
        the screen condition.

        """
        config = self.get_config()
        self.browser.get(ZOOMED_IN_MAPS_URL)
        # wait for compass to appear, then start testing by taking a screenshot
        tester = partial(exp_cond.visibility_of_element_located, (By.CLASS_NAME, "widget-compass"))
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(tester(), message="Compass widget did not appear.")

    @screenshot_on_error
    def test_zoom_out_button_change(self):
        """
        Test clicks on the zoom in button and checks the
        pose object coordinates (altitude, latitude, longitude)
        according change.

        """
        self._zoom_clicks_init()
        make_screenshot(self.browser, "zoom_out_button", 0)
        # get current values of altitude, latitude and longitude
        pose_start = self.get_camera_pose()
        self.click_zoom_out()
        pose = self.get_camera_pose()
        make_screenshot(self.browser, "zoom_out_button", 1)
        # the zoom out click increases altitude by approx 100% of the initial value
        # assume 10% difference from the target value to tolerate
        expected_pose = Pose(alt=pose_start.alt * 2,
                             lat=pose_start.lat,
                             lon=pose_start.lon)
        self.assert_pose_is_near(pose, expected_pose, alt_delta=pose.alt * 0.1)

    @screenshot_on_error
    def test_zoom_in_button_change(self):
        """
        Test clicks on the zoom out button and checks the
        pose object coordinates (altitude, latitude, longitude)
        according change.

        """
        self._zoom_clicks_init()
        make_screenshot(self.browser, "zoom_in_button", 0)
        # get current values of altitude, latitude and longitude
        pose_start = self.get_camera_pose()
        self.click_zoom_in()
        pose = self.get_camera_pose()
        make_screenshot(self.browser, "zoom_in_button", 1)
        # the zoom in click decreases altitude by approx 50% of the initial value
        # assume 10% difference from the target value to tolerate
        expected_pose = Pose(alt=pose_start.alt * 0.5,
                             lat=pose_start.lat,
                             lon=pose_start.lon)
        self.assert_pose_is_near(pose, expected_pose, alt_delta=pose.alt * 0.1)