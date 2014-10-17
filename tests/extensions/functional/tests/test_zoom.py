"""
Zoom buttons tests.

"""

from base import TestBaseTouchscreen
from base import MAPS_URL, ZOOMED_IN_MAPS_URL, Pose
from base import screenshot_on_error, make_screenshot
import helpers


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

    @screenshot_on_error
    def test_zoom_out_button_change(self):
        """
        Test clicks on the zoom in button and checks the
        pose object coordinates (altitude, latitude, longitude)
        according change.

        """
        helpers.wait_for_loaded_page(ZOOMED_IN_MAPS_URL, self.browser)
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
        assert self.pose_is_near(pose, expected_pose, alt_delta=pose.alt * 0.1) is True

    @screenshot_on_error
    def test_zoom_in_button_change(self):
        """
        Test clicks on the zoom out button and checks the
        pose object coordinates (altitude, latitude, longitude)
        according change.

        """
        helpers.wait_for_loaded_page(ZOOMED_IN_MAPS_URL, self.browser)
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
        assert self.pose_is_near(pose, expected_pose, alt_delta=pose.alt * 0.1) is True