"""
Tests related to Zoom buttons, zoom operation.

"""

import time

from base import TestBase
from base import Pose
import helpers


class TestZoomButtons(TestBase):
    """
    Tests for checking the zoom buttons are functional.

    """

    def setup_method(self, method):
        super(TestZoomButtons, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    @helpers.screenshot_on_error
    def test_zoom_buttons(self):
        """
        Test that the zoom in and out buttons are displayed.

        """
        self.browser.get(self.config["maps_url"])
        # this is the container for the two zoom buttons
        zoom = self.browser.find_element_by_id('zoom')
        assert zoom.is_displayed() is True
        for z in [zoom.find_element_by_class_name('widget-zoom-in'),
                  zoom.find_element_by_class_name('widget-zoom-out')]:
            assert z.is_displayed() is True

    @helpers.screenshot_on_error
    def test_zoom_out_button_change(self):
        """
        Test clicks on the zoom in button and checks the
        pose object coordinates (altitude, latitude, longitude)
        according change.

        """
        helpers.wait_for_loaded_page(self.config["zoomed_in_maps_url"], self.browser)
        # get current values of altitude, latitude and longitude
        pose_start = self.get_camera_pose()
        self.click_zoom_out()
        time.sleep(3)
        pose = self.get_camera_pose()
        # the zoom out click increases altitude by approx 100% of the initial value
        # assume 20% difference from the target value to tolerate
        expected_pose = Pose(alt=pose_start.alt * 2,
                             lat=pose_start.lat,
                             lon=pose_start.lon)
        assert self.pose_is_near(pose, expected_pose, alt_delta=pose.alt * 0.2) is True

    @helpers.screenshot_on_error
    def test_zoom_in_button_change(self):
        """
        Test clicks on the zoom out button and checks the
        pose object coordinates (altitude, latitude, longitude)
        according change.

        """
        helpers.wait_for_loaded_page(self.config["zoomed_in_maps_url"], self.browser)
        # get current values of altitude, latitude and longitude
        pose_start = self.get_camera_pose()
        self.click_zoom_in()
        time.sleep(3)
        pose = self.get_camera_pose()
        # the zoom in click decreases altitude by approx 50% of the initial value
        # assume 20% difference from the target value to tolerate
        expected_pose = Pose(alt=pose_start.alt * 0.5,
                             lat=pose_start.lat,
                             lon=pose_start.lon)
        assert self.pose_is_near(pose, expected_pose, alt_delta=pose.alt * 0.2) is True