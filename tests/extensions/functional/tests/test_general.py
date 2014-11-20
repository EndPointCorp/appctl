"""
Portal general selenium tests.

"""

import time
from functools import partial

import pytest
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.common.by import By
from selenium.webdriver.support import expected_conditions as exp_cond

from base import TestBaseTouchscreen
from base import MAPS_URL
from base import screenshot_on_error
import helpers


class TestSearch(TestBaseTouchscreen):
    """
    Search tests (search box, search button, etc).

    """

    @pytest.mark.skipif(True, reason="Unstable camera pose object attributes, reported.")
    @screenshot_on_error
    def test_search_hitting_return_on_search_box(self):
        """
        Test that positions were changed after
        putting something into the search box and hitting the return key.

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(MAPS_URL, self.browser)
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
                      config["max_load_timeout"]).until_not(tester, message=msg)

    @pytest.mark.skipif(True, reason="Unstable camera pose object attributes, reported.")
    @screenshot_on_error
    def test_search_hitting_return_on_search_button(self):
        """
        Test that positions were changed after
        putting something into the search box and clicking the search button.
        Well, this is idea, as commented below, clicking is actually not working
        likely to inability to get search button focus, so it's just sending
        enter key on the search button.

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(MAPS_URL, self.browser)
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
                      config["max_load_timeout"]).until_not(tester, message=msg)

    @pytest.mark.skipif(True, reason="Unstable camera pose object attributes, reported.")
    @screenshot_on_error
    def test_search_clicking_search_button(self):
        """
        Click event on the search button.

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(MAPS_URL, self.browser)
        pose_start = self.get_camera_pose()
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        self.click("searchbutton", finder="by_class")
        tester = lambda _: self.pose_is_near(pose_start,
                                             self.get_camera_pose(),
                                             assert_alt=False)
        msg = "Waiting for position change timed out."
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until_not(tester, message=msg)

    @screenshot_on_error
    def test_no_searchbox_on_other_planets(self):
        """
        The search box should not be visible when Moon, Mars are clicked.
        Start with out default maps URL, from there 1x click on zoom out
        button makes other universe objects (Mars, Moon) appear. Then
        clicking on Mars, Moon makes search box disappear, clicking on
        Earth back makes the search box appear.

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(MAPS_URL,
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
                      config["max_load_timeout"]).until(tester(), message=msg)

        # without this additional delay, clicking the planet just
        # doesn't seem to have effect (like if element is not fully loaded in DOM ...?)
        # not sure what to hook the condition to since the Mars, Moon labeled elements
        # are there but clicking them does nothing (without this delay)
        time.sleep(5)
        tray = self.browser.find_element_by_class_name("widget-runway-tray-wrapper")
        planets = tray.find_elements_by_class_name("widget-runway-card-button")

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
                      config["max_load_timeout"]).until_not(tester(), message=msg)
        box = self.browser.find_element_by_id("searchbox")
        assert box.is_displayed() is False
        # click on Earth now to make it appear
        # without this delay, the following click won't happen and the
        # explicit wait times out
        time.sleep(5)
        planets[0].click()
        msg = "Waiting for searchbox to appear (click on Earth) timed out."
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(tester(), message=msg)
        box = self.browser.find_element_by_id("searchbox")
        assert box.is_displayed() is True
        # click on Mars now to make it disappear
        # without this delay, the following click won't happen and the
        # explicit wait times out
        time.sleep(5)
        planets[2].click()
        msg = "Waiting for searchbox to disappear (click on Mars) timed out."
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until_not(tester(), message=msg)
        box = self.browser.find_element_by_id("searchbox")
        assert box.is_displayed() is False


class TestMiscellaneous(TestBaseTouchscreen):
    """
    Various other tests.

    """

    @pytest.mark.skipif(True, reason="Patch hiding the EU cookies message bar not yet merged.")
    @screenshot_on_error
    def test_eu_cookies_info_bar_is_hidden(self):
        """
        Make sure that the cookie bar is invisible.
        And the map canvas has set top=0px,
        because the cookie container is 30 px higher,
        and the canvas is moved 30 px down.
        https://redmine.endpoint.com/issues/2517
        Related patch:
        https://github.com/EndPointCorp/portal/commit/f7c89fecedd5bcaa94b03289b01393d8cfd9d692

        """
        helpers.wait_for_loaded_page(MAPS_URL, self.browser)
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
