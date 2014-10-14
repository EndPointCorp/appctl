"""
Portal general selenium tests.

"""

from selenium import webdriver
from selenium.webdriver.common.keys import Keys

from base import BaseTouchscreenTest
from base import MAPS_URL
from base import screenshot_on_error


class TestGeneral(BaseTouchscreenTest):
    """
    Test cases not fitting any other placement.

    """

    @screenshot_on_error
    def test_url_change_after_search_and_hitting_return(self):
        """
        Test that URL in the browser changes after
        putting something into the search box and hitting the return key.

        """
        self.browser.get(MAPS_URL)
        old_url = self.browser.current_url
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        # doesn't work if it's not on the correct element ...
        box.send_keys(Keys.RETURN)
        self.wait_for_url_change(old_url)
        # the URL changes with change of the content, test that
        assert old_url != self.browser.current_url

    @screenshot_on_error
    def test_url_change_after_search_and_clicking_search_button(self):
        """
        Test that URL in the browser changes after
        putting something into the search box and clicking the search button.

        """
        self.browser.get(MAPS_URL)
        old_url = self.browser.current_url
        box = self.browser.find_element_by_id("searchboxinput")
        box.send_keys("babice nad svitavou, czech republic")
        button = self.browser.find_element_by_class_name("searchbutton")
        action = webdriver.ActionChains(self.browser)
        action.move_to_element(button)
        action.perform()
        self.wait_for_url_change(old_url)
        # the URL changes with change of the content, test that
        assert old_url != self.browser.current_url