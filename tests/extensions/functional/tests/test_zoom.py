"""
Zoom buttons tests.

"""

from base import TestBaseTouchscreen
import time
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


    @screenshot_on_error
    def test_zoom_out_button_change(self):
        """
        Test clicks on the zoom in button and checks if the
        zoom level on the browser URL changed.

        """
        self.browser.get(ZOOMED_IN_MAPS_URL)
        make_screenshot(self.browser, "zoom_out_button", 0)

        time.sleep(10)
        pose = self.get_camera_pose()

        expected_pose = Pose(alt=620097.1867688844, lat=8.135687, lon=-75.0973243)

        #self.assert_pose_is_near(pose, expected_pose)
        self.click_zoom_out()

        make_screenshot(self.browser, "zoom_out_button", 1)
        current_pose = self.get_camera_pose()

        assert pose.alt < current_pose.alt
        #self.assert_pose_is_near(pose, expected_pose, assert_alt=False)


    @screenshot_on_error
    def test_zoom_in_button_change(self):
        """
        Test clicks on the zoom in button and checks if the
        zoom level on the browser URL changed.

        """
        self.browser.get(ZOOMED_IN_MAPS_URL)
        make_screenshot(self.browser, "zoom_in_button", 0)

        time.sleep(10)
        pose = self.get_camera_pose()

        expected_pose = Pose(alt=620097.1867688844, lat=8.135687, lon=-75.0973243)

        #self.assert_pose_is_near(pose, expected_pose)
        self.click_zoom_in()

        make_screenshot(self.browser, "zoom_in_button", 1)
        current_pose = self.get_camera_pose()
        assert pose.alt > current_pose.alt
        #self.assert_pose_is_near(pose, expected_pose, assert_alt=False)
