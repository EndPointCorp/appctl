"""
Tests for the display interface (display chrome extension).

"""

from functools import partial

from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as exp_cond

from base import TestBase
from base import MAPS_URL, ZOOMED_IN_MAPS_URL
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
            ("acme-poi-button", By.CLASS_NAME),
            ("widget-runway-tray-wrapper", By.CLASS_NAME),
            ("acme-points-of-interest", By.ID),
            ("acme-famous-places-button", By.ID),
            ("widget-runway-thumbstrip-background", By.CLASS_NAME),
            ("widget-compass", By.CLASS_NAME))


class TestBaseDisplay(TestBase):
    """
    Loads default extensions for display,
    all tests for display should inherit from this class.

    Test that the following graphical components are not displayed:
    zoom in/out buttons
    compass
    google More Fun menu
    search field
    search button
    runway panel

    """
    extensions = ["display", ]

    @screenshot_on_error
    def test_widgets_not_displayed(self):
        """
        After the browser loads the URL, it takes some time until
        also the extensions are fully loaded and the modify already
        loaded DOM. E.g. the graphical widgets (elements) in question
        are removed.

        """
        config = self.get_config()
        self.browser.get(MAPS_URL)

        def test_elements_not_present(elem):
            msg = "Element '{0}' should not be present (extension display).".format(elem[0])
            # An Expectation for checking that an element is either
            # invisible or not present on the DOM.
            tester = partial(exp_cond.invisibility_of_element_located, (elem[1], elem[0]))
            WebDriverWait(self.browser,
                          config["max_load_timeout"]).until(tester(), message=msg)
        map(test_elements_not_present, elements)
        make_screenshot(self.browser, "test_elements_not_present", 0)


class TestBaseKioskExtension(TestBase):
    """
    Test DOM elements are displayed with kiosk extension
    against them not displayed with the display extension.

    """
    extensions = ["kiosk", ]

    @screenshot_on_error
    def test_widgets_displayed(self):
        """
        If elements from the elements list are renamed, the test
        test_widgets_not_displayed() would just test that they are not present,
        however they might be, but under a different name.
        Test that exactly these elements are present when the display
        extension is not loaded.

        """
        config = self.get_config()
        self.browser.get(ZOOMED_IN_MAPS_URL)

        def test_elements_present(elem):
            msg = "Element '{0}' should be present (kiosk extension).".format(elem[0])
            tester = partial(exp_cond.visibility_of_element_located, (elem[1], elem[0]))
            WebDriverWait(self.browser,
                          config["max_load_timeout"]).until(tester(), message=msg)
        map(test_elements_present, elements)
        make_screenshot(self.browser, "test_widgets_displayed", 0)
