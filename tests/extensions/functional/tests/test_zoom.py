"""
Zoom buttons tests.

"""

from base import BaseTouchscreenTest
from base import MAPS_URL
from base import screenshot_on_error, make_screenshot


class TestZoomButtons(BaseTouchscreenTest):
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
        self.browser.get(MAPS_URL)
        make_screenshot(self.browser, "zoom_out_button", 0)
        zoom_level = self.get_current_zoom_level()
        self.click_zoom_out()
        make_screenshot(self.browser, "zoom_out_button", 1)
        assert zoom_level < self.get_current_zoom_level()

    @screenshot_on_error
    def test_zoom_in_button_change(self):
        """
        Test clicks on the zoom in button and checks if the
        zoom level on the browser URL changed.

        """
        self.browser.get(MAPS_URL)
        make_screenshot(self.browser, "zoom_in_button", 0)
        zoom_level = self.get_current_zoom_level()
        self.click_zoom_in()
        make_screenshot(self.browser, "zoom_in_button", 1)
        assert zoom_level > self.get_current_zoom_level()
