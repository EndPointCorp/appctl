"""
The runway is the element at to bottom of the touchscreen browser,
it contains a list of Points of Interests
(if the camera is zoomed in to some level, and there are some
interesting places around) or the list of planets to load (Earth, Moon, Mars).

There is also a list of Famous Places, which is always filled,
as we load there a static list of entries read from a file.

The tests should check if:
- there are POI and Famous Places buttons
- user can scroll both lists with touch events (for the tests we need to use
  the touch events, not mouse ones)
- there is Mars, Earth, Moon loaded when tapped a proper picture and the
  camera is maximally zoomed out
- an POI is loaded, and we can exit it, and load another one

"""


from selenium.webdriver.common.by import By

from base import MAPS_URL, ZOOMED_IN_MAPS_URL
from base import screenshot_on_error, make_screenshot
from base import TestBase
import helpers


class TestRunway(TestBase):

    extensions = ["kiosk"]

    @screenshot_on_error
    def test_runway_buttons(self):
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
        # TODO:
        # how to generate "touch events"?
        # the scroll / tray div is like this:
        # <div jsaction="DOMMouseScroll:runway.tray;mousewheel:runway.tray" class="widget-runway-tray-wrapper"
        #  jstcache="0"> <ul class="widget-runway-subview-card-list widget-runway-all-cards" ...

    @screenshot_on_error
    def test_planets_on_max_zoom_out(self):
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
        tray = self.browser.find_element_by_class_name("widget-runway-tray-wrapper")
        planets = tray.find_elements_by_class_name("widget-runway-card-button")
        # since the other tab, Famous Places currently has 22 items, this tested
        # Points of Interest tab/tray has 22 items too.
        # Only the first 3 are set though with planets icons.
        for i in range(0, 3):
            assert planets[i].text in ("Earth", "Moon", "Mars")
        # TODO:
        # continue testing to select all planets, zoom stays maximal

    @screenshot_on_error
    def test_famous_places_displayed(self):
        """
        Check there are icons for famous places set, non empty text.

        """
        pass

    @screenshot_on_error
    def test_points_of_interest(self):
        """
        Test Points of Interest is loaded, we can load it, exit it and
        and load another Point of Interest.

        """
        pass