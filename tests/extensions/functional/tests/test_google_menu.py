"""
Google Menu related tests.

"""

import re

import pytest
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.common.by import By

from base import TestBaseTouchscreen
from base import screenshot_on_error
import helpers


class TestGoogleMenu(TestBaseTouchscreen):
    """
    Google Menu tests.

    """

    @screenshot_on_error
    def test_google_menu_is_visible(self):
        """
        Test that Google Menu (More fun) is displayed along with some items.

        """
        config = self.get_config()
        self.browser.get(config["maps_url"])
        morefun = self.browser.find_element_by_id('morefun')
        assert morefun.is_displayed() is True
        items = self.browser.find_element_by_id('morefun_items')
        assert items.is_displayed() is False

    @screenshot_on_error
    def test_google_items_are_visible_on_click(self):
        """
        Test that Google Menu (More fun) items are visible after clicking it.

        """
        config = self.get_config()
        self.browser.get(config["maps_url"])
        morefun = self.browser.find_element_by_id('morefun')
        morefun.click()
        assert morefun.is_displayed() is True
        items = self.browser.find_element_by_id('morefun_items')
        assert items.is_displayed() is True

    @pytest.mark.skipif(True, reason="More fun -> Maps, Timelapse click doesn't do anything, #200")
    @screenshot_on_error
    def test_clicking_doodle_item(self):
        """
        Test that clicking on the doodle item changes the URL to the
        doodles page.

        """
        config = self.get_config()
        helpers.wait_for_loaded_page(config["zoomed_in_maps_url"],
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="morefun")
        # get it to be able to click on it
        morefun = self.browser.find_element_by_id('morefun')
        morefun.click()
        items = self.browser.find_element_by_id('morefun_items')
        li_items = items.find_elements_by_tag_name('li')
        assert len(li_items) == 2
        doodle = li_items[1]
        doodle.click()

        def tester(_):
            print self.browser.current_url
            if re.match(r'chrome-extension:\/\/[a-z]+\/pages\/doodles.html',
                        self.browser.current_url):
                return True
            else:
                return False
        # need to wait, sometimes it's not there immediately
        msg = "Waiting for doodle URL to appear timed-out."
        WebDriverWait(self.browser,
                      config["max_load_timeout"]).until(tester, message=msg)