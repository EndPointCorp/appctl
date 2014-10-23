"""
Portal general selenium tests.

"""

from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait

from base import TestBaseTouchscreen
from base import MAPS_URL
from base import screenshot_on_error
import helpers


class TestSearch(TestBaseTouchscreen):
    """
    Search tests (search box, search button, etc).

    """

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