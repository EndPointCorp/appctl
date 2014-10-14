"""
Portal selenium tests base module.

TODO:
variables / constants such as MAP_URL
directory paths should go in the config.json file, discuss

"""


from selenium import webdriver
import os
from functools import wraps
import traceback
from datetime import datetime
import logging
import time
import os
import json
import pprint


# The selenium logger is just too noisy, it shows the whole terribly huge
# base64 encoded json message, which contains the whole visible map,
# and it has over 10000 lines on my console, so it will be hidden for now.
# The huge info is shown as logging.DEBUG message.
logging.getLogger('selenium.webdriver.remote').setLevel(logging.ERROR)

# This one will be hardcoded for now
MAPS_URL = 'https://www.google.com/maps/@8.135687,-75.0973243,17856994a,40.4y,1.23h/data=!3m1!1e3?esrch=Tactile::TactileAcme'

# We need another url for zoom out button, the above one cannot be zoomed out
ZOOMED_IN_MAPS_URL = 'https://www.google.com/maps/@8.135687,-75.0973243,178569a,40.4y,1.23h/data=!3m1!1e3?esrch=Tactile::TactileAcme'

# Below directories are relative from the __file__ directory, and
# point to those in the main directory
EXTENSIONS_DIR = "../../../extensions"
SCREENSHOTS_DIR = "../../../tests_results"

# configuration file, from the project root
CONFIG_FILE = os.path.join("..", "..", "..", "config.json")
# configuration data instance, keep here to have it valid
# during the entire test suite run
CONFIG = {}


# TODO: add function to make slingshot on purpose -
# maybe modify the lg-grap-screens
# the slingshot is at /tmp/montage/lg-montage.png

# TODO: add function to make screenshot on purpose

def make_screenshot(browser, fname, index, dir=SCREENSHOTS_DIR):

    if not os.path.exists(dir):
        os.makedirs(dir)

    fname = "{}{}{}_{}_{}".format(dir,
                                  os.path.sep,
                                  datetime.now().isoformat(),
                                  fname,
                                  index)
    #print "Making screenshot: " + fname
    browser.save_screenshot('{}.png'.format(fname))
    return fname

def screenshot_on_error(test):
    """
    Annotation for test functions to make screenshots on error.

    The wrapper runs the test. When it fails, then it makes a screenshot
    in the SCREENSHOTS_DIR

    Usage:
        @screenshot_on_error
        def test_something(self):
            pass

    """
    @wraps(test)
    def wrapper(*args, **kwargs):
        try:
            test(*args, **kwargs)
        except:
            test_obj = args[0]
            test_name = test_obj.current_method
            fname = make_screenshot(test_obj.browser, test_name, 0)
            with open("{}.log".format(fname), "w") as flog:
                flog.write(traceback.format_exc())
            raise
    return wrapper

from collections import namedtuple
Pose = namedtuple("pose", ['alt', 'lon', 'lat'])

class BaseTest(object):
    """
    Base unit test class, with all the utilities for starting Chrome.

    Test suite configuration is loaded once per test suite from here
    with this class initialization.

    """

    @classmethod
    def setup_class(cls):
        """
        Setup any state specific to the execution of the given module.

        """
        global CONFIG_FILE, CONFIG
        if CONFIG:
            print "\nCONFIG is initialized, do not load"
        else:
            print "\nCONFIG is NOT initialized, loading from '%s' ..." % CONFIG_FILE
            f = open(CONFIG_FILE, 'r')
            CONFIG = json.load(f)
            print "CONFIG: test suite configuration:"
            pprint.pprint(CONFIG)

    @classmethod
    def set_environ_driver_path(cls, driver_path):
        """
        Sets environment variable for Chrome.

        Chrome driver needs to have an environment variable set, this must be
        set to the path to the webdrver file.

        Args:
            driver_path: path for the driver to be set

        Returns:
            Nothing

        """
        os.environ["webdriver.chrome.driver"] = driver_path

    @classmethod
    def get_extensions_options(cls, extensions):
        """
        Returns ChromeOptions object with extensions paths.

        The Chrome browser started by Selenium doesn't have any plugins loaded.
        We need plugins for kiosk and display, so we need to provide the list
        in the ChromeOptions object.

        The path can be relative, however it needs to load packed extensions.

        Args:
            extensions: extension names to be loaded

        Returns:
            ChromeOptions object with extensions declarations

        """
        op = webdriver.ChromeOptions()
        for ext in extensions:
            op.add_extension('{}/{}.crx'.format(EXTENSIONS_DIR, ext))
        return op

    @classmethod
    def run_browser(cls):
        """
        Runs browser with proper driver path and extensions.

        Returns:
            selenium driver handler

        """
        driver = CONFIG["chromedriver"]["path"]
        options = cls.get_extensions_options(cls.extensions)

        cls.set_environ_driver_path(driver)
        return webdriver.Chrome(executable_path=driver,
                                chrome_options=options)

    def setup_method(self, method):
        self.browser = self.run_browser()
        self.current_method = method.__name__

    def teardown_method(self, _):
        self.browser.quit()

    def get_camera_pose(self):
        """ TODO: add the angles """
        res = self.browser.execute_script('return acme.getCameraPose();')
        return Pose(res['alt'], res['g'], res['wg'])

    def click(self, finder_value, finder):
        """
        Performs javascript click on the element.

        Arguments:
            finder_value: string to find
            finder: "by_class"

        A kind of workaround to a bug:
        https://code.google.com/p/selenium/issues/detail?id=6218

        """
        if finder == "by_class":
            finder_script = """
            var element = document.getElementsByClassName('{0}')[0];
            if (element == null) {{
                alert('Didn\\'t find the element with class = {0}');
                return;
            }}
            """.format(finder_value)
        else:
            raise ValueError("Wrong finder value")

        event_script = "(function(){{ {0} ; element.click(); }}());" \
                       .format(finder_script)
        self.browser.execute_script(event_script)

    def click_zoom_in(self):
        """
        Performs click on the zoom-in button.
        Waits for a moment for the browser to change URL.
        If the browser URL didn't change, then the function ends without
        any problem.

        """
        old_url = self.browser.current_url
        self.click('widget-zoom-in', finder="by_class")
        self.wait_for_url_change(old_url)

    def click_zoom_out(self):
        """
        Performs click on the zoom-out button.
        Waits for a moment for the browser to change URL.
        If the browser URL didn't change, then the function ends without
        any problem.

        """
        old_url = self.browser.current_url
        self.click('widget-zoom-out', finder="by_class")
        self.wait_for_url_change(old_url)

    def get_current_zoom_level(self):
        """
        Get the current zoom level from the browser's url.

        """
        return self._get_zoom_level_from_url(self.browser.current_url)

    def _get_zoom_level_from_url(self, url):
        """
        Returns zoom level from the given url.

        Sample urls, chrome switches from one to another from time to time:
            https://www.google.com/maps/@8.135687,-75.0973243,17856994a,40.4y,1.23h/data=!3m1!1e3
            https://www.google.com/maps/@8.135687,-75.0973243,18207688m/data=!3m1!1e3

        Arguments:
            url - url taken from the browser on maps.google.com

        Returns:
            (int) zoom level read from the url

        """
        import re
        return int(re.search("\/maps\/.*,(\d+)[am][,/]", url).groups()[0])

    def wait_for_url_change(self, old_value, interval=1, max_wait_time=20):
        """
        Checks periodically for the browser url to change.

        This is a kind of active wait. The function runs at most for the
        {max_wait_time}, but can return earlier, if the conditions is met.

        This function returns regardless the url changed. It's up to test maker
        to implement checks after calling this function.

        Function checks if the browser url changed.
        If changed, then the function returns immediately.
        If not changed, then the function goes to sleep for {interval} time.

        If the function runs for more than the {max_wait_time} then it returns.

        Arguments:
            old_value - old value for the url, checked agains the current url
            interval  - sleep time between checks
            max_wait_time - maximum time for the function to run

        Returns:
            None

        """
        start = time.time()
        while (start + max_wait_time > time.time()):
            #print self.browser.current_url
            if old_value != self.browser.current_url:
                return
            time.sleep(interval)


class BaseDisplayTest(BaseTest):
    """
    Loads default extensions for display,
    all tests for display should inherit from this class.

    """
    extensions = ["display", ]


class BaseTouchscreenTest(BaseTest):
    """
    Loads default extensions for touchscreen,
    all tests for touchscreen should inherit from this class.

    """
    extensions = ["kiosk", ]
