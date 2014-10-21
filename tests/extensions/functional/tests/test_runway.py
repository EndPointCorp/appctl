"""
The runway is the element at to bottom of the touchscreen browser,
it contains a list of Points of Interests
(if the camera is zoomed in to some level, and there are some
interesting places around) or the list of planets to load (Earth, Moon, Mars).

There is also a list of Famous Places, which is always filled,
as we load there a static list of entries read from a file.

The tests check:
- there are POI and Famous Places buttons
- user can scroll both lists with touch events (for the tests we need to use
  the touch events, not mouse ones)
- there is Mars, Earth, Moon loaded when tapped a proper picture and the
  camera is maximally zoomed out
- an POI is loaded, and we can exit it, and load another one

"""


import time

from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys

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
        # how to generate "touch events" to scroll the tray div:
        # <div jsaction="DOMMouseScroll:runway.tray;mousewheel:runway.tray" class="widget-runway-tray-wrapper"
        #  jstcache="0"> <ul class="widget-runway-subview-card-list widget-runway-all-cards" ...

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
        time.sleep(1)
        # now select Points of Interest and check the planets are shown
        poi = self.browser.find_element_by_id("acme-poi-button")
        poi.click()
        tray = self.browser.find_element_by_class_name("widget-runway-tray-wrapper")
        planets = tray.find_elements_by_class_name("widget-runway-card-button")
        # since the other tab, Famous Places currently has 22 items, this tested
        # Points of Interest tab/tray has 22 items too.
        # Only the first 3 are set though with planets icons.
        for i in range(0, 3):
            try:
                assert planets[i].text in ("Earth", "Moon", "Mars")
                # test selecting planets, zoom stays maximal
                planets[i].click()
                time.sleep(1)
                assert zoom_out_button.is_enabled() is False
            except AssertionError:
                print "AssertionError"
                for ii in range(len(planets)):
                    print "%s .. %s" % (ii, planets[ii].text)
                    # let it fail again:
                    assert planets[i].text in ("Earth", "Moon", "Mars")

    @screenshot_on_error
    def test_runway_points_of_interest(self):
        """
        Test Points of Interest is loaded, we can load one, exit it and
        and load another Point of Interest.

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
        time.sleep(4)
        tray = self.browser.find_element_by_class_name("widget-runway-tray-wrapper")
        points = tray.find_elements_by_class_name("widget-runway-subview-card-view-container")
        print "Found %s POIs" % len(points)
        c = 0
        for p in points:
            action = webdriver.ActionChains(self.browser)
            action.move_to_element(p).click().perform()
            time.sleep(3)
            eb = self.browser.find_element_by_class_name("widget-titlecard-exitcontainer")
            action = webdriver.ActionChains(self.browser)
            action.move_to_element(eb).click().perform()
            if c > 5:
                break
            c += 1