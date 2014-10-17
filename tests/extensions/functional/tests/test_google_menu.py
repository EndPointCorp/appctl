"""
Google Menu tests
"""


from base import TestBaseTouchscreen
import time
from base import MAPS_URL, ZOOMED_IN_MAPS_URL, Pose
from base import screenshot_on_error, make_screenshot
import re

class TestGoogleMenu(TestBaseTouchscreen):

    @screenshot_on_error
    def test_google_menu_is_visible(self):
        self.browser.get(MAPS_URL)
        morefun = self.browser.find_element_by_id('morefun')
        assert morefun.is_displayed() is True
        items = self.browser.find_element_by_id('morefun_items')
        assert items.is_displayed() is False

    @screenshot_on_error
    def test_google_items_are_visible_on_click(self):
        self.browser.get(MAPS_URL)
        morefun = self.browser.find_element_by_id('morefun')
        morefun.click()
        assert morefun.is_displayed() is True
        items = self.browser.find_element_by_id('morefun_items')
        assert items.is_displayed() is True

    @screenshot_on_error
    def test_clicking_doodle_item(self):
        "Clicking on the doodle item should change the url to the doodles page"
        self.browser.get(ZOOMED_IN_MAPS_URL)
        time.sleep(5)
        morefun = self.browser.find_element_by_id('morefun')
        morefun.click()
        items = self.browser.find_element_by_id('morefun_items')
        li_items = items.find_elements_by_tag_name('li')
        assert len(li_items) == 2
        doodle = li_items[1]
        doodle.click()

        assert re.match(r'chrome-extension:\/\/[a-z]+\/pages\/doodles.html',
                        self.browser.current_url)



