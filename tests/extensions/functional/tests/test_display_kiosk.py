"""
The display extension tests.

The kiosk extension tests.

Cross-verification of DOM elements between the display and kiosk extensions.

"""

from functools import partial

import pytest
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as exp_cond

from base import TestBase
from base import screenshot_on_error, make_screenshot


# these particular elements are present when the display extension is not loaded
# some are IDs, some are class names - as stated lookup method
elements = (("zoom", By.ID),
            ("widget-zoom-button", By.CLASS_NAME),
            ("widget-zoom-in", By.CLASS_NAME),
            ("widget-zoom-out", By.CLASS_NAME),
            ("searchboxinput", By.ID),
            ("searchbox_form", By.ID),
            ("searchbutton", By.CLASS_NAME),
            ("morefun", By.ID),
            ("acme-poi-button", By.CLASS_NAME),
            ("widget-runway-tray-wrapper", By.CLASS_NAME),
            ("acme-points-of-interest", By.ID),
            ("acme-famous-places-button", By.ID),
            ("widget-runway-thumbstrip-background", By.CLASS_NAME),
            ("widget-compass", By.CLASS_NAME))


class TestBaseDisplay(TestBase):
    """
    Tests related to the display extension.

    """

    def setup_method(self, method):
        super(TestBaseDisplay, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["display"])

    @pytest.mark.skipif(True, reason=("Fails sometimes. Need explanation of the display "
                                      "loading. Details on #201"))
    @screenshot_on_error
    def test_widgets_not_displayed(self):
        """
        Test that the following graphical widgets (elements) are not displayed:
            * zoom in/out buttons,
            * compass,
            * google More Fun menu,
            * search field,
            * search button,
            * runway panel (Points of Interest, Famous Places).

        After the browser loads the initial URL, it takes some time
        until also the extensions are fully loaded and until the DOM is
        modified accordingly.

        """
        self.browser.get(config["maps_url"])

        def test_elements_not_present(elem):
            msg = "Element '{0}' should not be present (extension display).".format(elem[0])
            # An Expectation for checking that an element is either
            # invisible or not present on the DOM.
            tester = partial(exp_cond.invisibility_of_element_located, (elem[1], elem[0]))
            WebDriverWait(self.browser,
                          self.config["max_load_timeout"]).until(tester(), message=msg)
        map(test_elements_not_present, elements)
        make_screenshot(self.browser, "test_elements_not_present", 0)


class TestBaseKiosk(TestBase):
    """
    Tests related to the kiosk extension.

    """

    def setup_method(self, method):
        super(TestBaseKiosk, self).setup_method(method)
        self.browser = self.run_browser(self.config["chromes"]["kiosk"])

    @screenshot_on_error
    def test_widgets_displayed(self):
        """
        Test that the following graphical widgets (elements) are displayed:
            * zoom in/out buttons,
            * compass,
            * google More Fun menu,
            * search field,
            * search button,
            * runway panel (Points of Interest, Famous Places).

        If elements from the elements list are renamed, the test
        **test_widgets_not_displayed()** would just test that they are not present,
        however they might be, but under a different name.
        Test that exactly the same are present when the display
        extension is not loaded, i.e. with kiosk extension loaded.

        """
        self.browser.get(self.config["zoomed_in_maps_url"])

        def test_elements_present(elem):
            msg = "Element '{0}' should be present (kiosk extension).".format(elem[0])
            tester = partial(exp_cond.visibility_of_element_located, (elem[1], elem[0]))
            WebDriverWait(self.browser,
                          self.config["max_load_timeout"]).until(tester(), message=msg)
        map(test_elements_present, elements)
        make_screenshot(self.browser, "test_widgets_displayed", 0)
