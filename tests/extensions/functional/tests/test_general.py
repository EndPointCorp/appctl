"""
Portal general selenium tests.

"""

from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait

from base import TestBaseTouchscreen
from base import MAPS_URL
from base import screenshot_on_error
import helpers


class TestGeneral(TestBaseTouchscreen):
    """
    Test cases not fitting any other placement.

    """

    @screenshot_on_error
    def test_search_hitting_return(self):
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