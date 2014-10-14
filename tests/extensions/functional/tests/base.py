"""
Portal selenium tests base module.

"""


from selenium import webdriver
from functools import wraps
import traceback
from datetime import datetime
import logging
import time
import os
import json
import pprint
from collections import namedtuple
import re

import pytest


# The selenium logger is just too noisy, it shows the whole terribly huge
# base64 encoded json message, which contains the whole visible map,
# and it has over 10000 lines on my console, so it will be hidden for now.
# The huge info is shown as logging.DEBUG message.
logging.getLogger('selenium.webdriver.remote').setLevel(logging.ERROR)

# This one will be hardcoded for now
MAPS_URL = 'https://www.google.com/maps/@8.135687,-75.0973243,17856994a,40.4y,1.23h/data=!3m1!1e3?esrch=Tactile::TactileAcme'

# We need another url for zoom out button, the above one cannot be zoomed out
ZOOMED_IN_MAPS_URL = 'https://www.google.com/maps/@8.135687,-75.0973243,178569a,40.4y,1.23h/data=!3m1!1e3?esrch=Tactile::TactileAcme'

# configuration data instance, keep here to have it valid
# during the entire test suite run, it's initialized just
# once per entire run
CONFIG = {}

Pose = namedtuple("pose", ['alt', 'lon', 'lat'])


# TODO: add function to make slingshot on purpose -
# maybe modify the lg-grap-screens
# the slingshot is at /tmp/montage/lg-montage.png

# TODO: add function to make screenshot on purpose

def make_screenshot(browser, fname, index):
    ss_dir = CONFIG["screenshots_dir"]

    if not os.path.exists(ss_dir):
        os.makedirs(ss_dir)

    fname = "{}{}{}_{}_{}".format(ss_dir,
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

    The wrapper runs the test. When it fails, then it makes
    a screenshot in the CONFIG["screenshots_dir"] directory.

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


def load_configuration():
    """
    Read the test suite JSON configuration from a file.
    The file path is read from a env variable.
    Set the global CONFIG object.

    :return: nothing

    """
    global CONFIG
    portal_config_env = "PORTAL_TESTS_CONFIG"
    try:
        config_file = os.environ[portal_config_env]
    except KeyError:
        m = "Can't load tests config file, set '%s' accordingly." % portal_config_env
        pytest.exit(m)

    print ("Loading configuration from "
           "env PORTAL_TESTS_CONFIG: '%s' ..." % config_file)
    f = open(config_file, 'r')
    CONFIG = json.load(f)
    print "CONFIG: test suite configuration:"
    pprint.pprint(CONFIG)


def set_env_variables():
    """
    Sets env variables according to values from the config file.

    :return: nothing

    """
    global CONFIG
    if "env_vars" in CONFIG:
        print "Setting env. variables ..."
        for var in CONFIG["env_vars"]:
            name = var["name"]
            old = os.environ.get(name, None)
            if var["extend_current"] and old:
                os.environ[name] = old + ":" + var["value"]
            else:
                os.environ[name] = var["value"]
            print ("Set env '%s' old value: '%s' current value: '%s'" %
                  (name, old, os.environ[name]))


def prepare_environment():
    """
    Load the test suite configuration and prepare
    the enviroment.
    Run executables before the test suite requires.

    :return: nothing

    """
    global CONFIG
    if CONFIG:
        print "\nCONFIG is initialized, do not load anything ..."
    else:
        print "\nCONFIG is NOT initialized, running preparation ..."
        load_configuration()
        set_env_variables()
        for command in CONFIG["executables"]:
            print "Running '%s' ..." % command
            r = os.system(command)
            print "exit status: %s" % r


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
        prepare_environment()


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
            op.add_extension('{}/{}.crx'.format(CONFIG["extensions_dir"], ext))
        return op

    @classmethod
    def run_browser(cls):
        """
        Runs browser with proper driver path and extensions.

        Returns:
            selenium driver handler

        """
        driver = CONFIG["chrome_driver"]["path"]
        options = cls.get_extensions_options(cls.extensions)
        # Set environment variable for Chrome.
        # Chrome driver needs to have an environment variable set,
        # this must be set to the path to the webdriver file.
        os.environ["webdriver.chrome.driver"] = driver
        return webdriver.Chrome(executable_path=driver, chrome_options=options)

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
