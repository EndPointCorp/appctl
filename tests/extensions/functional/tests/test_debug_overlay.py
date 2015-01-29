
from selenium.webdriver.common.by import By
from base import TestBase
import helpers
import time
from selenium.webdriver.common.touch_actions import TouchActions
from selenium.webdriver.common.action_chains import ActionChains



class TestDebugOverlay(TestBase):

    def setup_method(self, method):
        super(TestDebugOverlay, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    def teardown_method(self, _):
        self.browser.quit()

    def find_elements(self):
        """
            Searches for all the html elements we need to check in this test.
        """
        self.do = self.browser.find_element_by_id('debug_overlay')
        self.do_p = self.browser.find_element_by_id('debug_overlay_password')
        self.mf = self.browser.find_element_by_id('morefun')
        self.items = self.browser.find_element_by_id('morefun_items')
        self.p = self.browser.find_element_by_id('password')

    def press(self, element, tap_time=0, event_type="click"):
        """
            Simulates pressing the element.
            element    - element
            tap_time   - time between pressing and releasing the left mouse button
            event_type - type of the events sent ["touch", "click"]
        """
        if event_type == "touch":
            x, y = element.location['x'], element.location['y']
            tap = TouchActions(self.browser)
            tap.tap_and_hold(x, y).perform()
            time.sleep(tap_time)
            tar = TouchActions(self.browser)
            tar.release(x, y).perform()
        elif event_type == "click":
            tap = ActionChains(self.browser)
            tap.move_to_element(element).click_and_hold(element).perform()
            time.sleep(tap_time)
            tar = ActionChains(self.browser)
            tar.move_to_element(element).release(element).perform()
        else:
            raise ValueError("event_type can be click or touch")

    def wait_for_loading_morefun(self):
        """
            For the debug overlay tests we are interested only in loading the morefun button.
        """
        helpers.wait_for_loaded_page(self.config["maps_url"],
                                     self.browser,
                                     elem_identifier_kind=By.ID,
                                     elem_identifier_name="morefun")


    def run_both_events(self, fn):
        """
            Runs the given function twice, once with each argument: 'click', 'touch'.
            :param fn: function to be run
        """
        fn('touch')
        fn('click')

    @helpers.screenshot_on_error
    def test_not_showing_password_field(self):
        """
            If the morefun press is less than 10 seconds, only items should be shown.
        """
        def run(event):
            self.wait_for_loading_morefun()
            self.find_elements()
            self.press(self.mf, 5, event)

            assert self.mf.is_displayed() is True
            assert self.items.is_displayed() is True
            assert self.do.is_displayed() is False
            assert self.do_p.is_displayed() is False

        self.run_both_events(run)

    @helpers.screenshot_on_error
    def test_showing_password_field(self):
        """
            If the morefun press is more than 10 seconds,then the overlay password field should be shown.
        """
        def run(event):
            self.wait_for_loading_morefun()
            self.find_elements()
            self.press(self.mf, 15)

            assert self.mf.is_displayed() is True
            assert self.items.is_displayed() is False
            assert self.do.is_displayed() is False
            assert self.do_p.is_displayed() is True

        self.run_both_events(run)

    @helpers.screenshot_on_error
    def test_entering_bad_password(self):
        """
            Entering bad password for the password field should just hide the password overlay.
        """

        def run(event):
            self.wait_for_loading_morefun()
            self.find_elements()
            for key in ['123', '', 'alkdjlkasj', '340sakjdk']:
                self.press(self.mf, 15, event)
                assert self.do_p.is_displayed() is True
                self.p.send_keys(key+"\n")
                time.sleep(3)
                assert self.do_p.is_displayed() is False

        self.run_both_events(run)

    @helpers.screenshot_on_error
    def test_entering_good_password(self):
        """
            Entering good password should hide the password overlay and show the debug one.
        """

        def run(event):
            self.wait_for_loading_morefun()
            self.find_elements()
            self.press(self.mf, 15, event)
            assert self.do_p.is_displayed() is True
            self.p.send_keys("slinkydog\n")
            time.sleep(3)
            assert self.do_p.is_displayed() is False
            assert self.do.is_displayed() is True

        self.run_both_events(run)
