"""
Google Menu related tests.

"""

import re

from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.common.by import By

from base import TestBase
import helpers


class TestGoogleMenu(TestBase):
    """
    Google Menu tests.

    """

    def setup_method(self, method):
        super(TestGoogleMenu, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    @helpers.screenshot_on_error
    def test_google_menu_is_visible(self):
        """
        Test that Google Menu (More fun) is displayed along with some items.

        """
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="morefun")
        morefun = self.browser.find_element_by_id('morefun')
        assert morefun.is_displayed() is True
        items = self.browser.find_element_by_id('morefun_items')
        assert items.is_displayed() is False

    @helpers.screenshot_on_error
    def test_google_items_are_visible_on_click(self):
        """
        Test that Google Menu (More fun) items are visible after clicking it.

        """
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="morefun")
        morefun = self.browser.find_element_by_id('morefun')
        assert morefun.is_displayed() is True
        morefun.click()
        items = self.browser.find_element_by_id('morefun_items')
        assert items.is_displayed() is True

    @helpers.screenshot_on_error
    def test_clicking_doodle_item(self):
        """
        Test that clicking on the doodle item changes the URL to the
        doodles page.

        """
        helpers.wait_for_loaded_page(self.config["zoomed_in_maps_url"],
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
            #print self.browser.current_url
            if re.match(r'chrome-extension:\/\/[a-z]+\/pages\/doodles.html',
                        self.browser.current_url):
                return True
            else:
                return False
        # need to wait, sometimes it's not there immediately
        msg = "Waiting for doodle URL to appear timed-out."
        WebDriverWait(self.browser,
                      self.config["max_load_timeout"]).until(tester, message=msg)
