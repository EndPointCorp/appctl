"""
Portal selenium tests base module.

"""


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

from selenium import webdriver
import pytest


# The selenium logger is just too noisy, it shows the whole terribly huge
# base64 encoded json message, which contains the whole visible map,
# and it has over 10000 lines on my console, so it will be hidden for now.
# The huge info is shown as logging.DEBUG message.
logging.getLogger('selenium.webdriver.remote').setLevel(logging.ERROR)

# configuration data instance, keep here to have it valid
# during the entire test suite run, it's initialized just
# once per entire run
CONFIG = {}

# Class used for runing information from the acme.getCurrentPose() function
Pose = namedtuple("pose", ['alt', 'lon', 'lat'])


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
    Decorator for test functions to make screenshots on error.

    The wrapper runs the test. When it fails, then it makes
    a screenshot in the CONFIG["screenshots_dir"] directory.

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
    # for jenkins, the following variables are missing from the configuration
    # file, so figure them out and set into CONFIG accordingly
    config_vars = dict(extensions_dir="extensions", screenshots_dir="tests_results")
    for var_key, var_value in config_vars.items():
        if not CONFIG.get(var_key, None):
            # this will fail if not set up properly
            workspace = os.environ["WORKSPACE"]
            CONFIG[var_key] = os.path.join(workspace, var_value)

    print "CONFIG: test suite configuration:"
    pprint.pprint(CONFIG)
    return CONFIG


def set_env_variables():
    """
    Sets env variables according to values from the config file.

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
    the environment.
    Run executables before the test suite requires.

    """
    global CONFIG
    if CONFIG:
        print "\nCONFIG is initialized, not loading anything ..."
    else:
        print "\nCONFIG is NOT initialized, running preparation ..."
        load_configuration()
        set_env_variables()
        for command in CONFIG["executables"]:
            print "Current working directory: '%s'" % os.getcwd()
            print "Running '%s' ..." % command
            r = os.system(command)
            print "exit status: %s" % r


class TestBase(object):
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
    def get_config(cls):
        """
        Return the configuration object.
        If client module does just import, it will import the symbol
        before it is initialized, hence empty.

        Returns:
            configuration object

        """
        global CONFIG
        return CONFIG

    @classmethod
    def _get_extensions_options(cls, config_chrome_section):
        """
        Returns ChromeOptions object with extensions paths.

        The Chrome browser started by Selenium doesn't have any plugins loaded.
        We need plugins for kiosk and display, so we need to provide the list
        in the ChromeOptions object.

        The path can be relative, however it needs to load packed extensions.

        Args:
            config_chrome_section: corresponding chrome section from the
                configuration file describing the chrome configuration
                we want to load

        Returns:
            ChromeOptions object with extensions declarations

        """
        op = webdriver.ChromeOptions()
        # if the path doesn't exist, let it fail, being clever here
        # could lead to some unexpected surprises of picking up undesired browser
        op.binary_location = config_chrome_section["binary_path"]

        for chrome_argument in config_chrome_section["arguments"]:
            op.add_argument(chrome_argument)

        for ext_name in config_chrome_section["extensions"]:
            ext_dir = CONFIG["extensions_dir"]
            # be less verbose now
            #print "Loading extension {} from {}".format(ext_name, ext_dir)
            op.add_extension('{}/{}.crx'.format(ext_dir, ext_name))
        return op

    @classmethod
    def run_browser(cls, config_chrome_section):
        """
        Runs browser with proper driver path and extensions.

        Args:
            config_chrome_section: corresponding chrome section from the
                configuration file describing the chrome configuration
                we want to load

        Returns:
            selenium browser driver handler

        """
        # print "Starting browser for configuration (from \"chromes\" section):"
        # pprint.pprint(config_chrome_section)
        capabilities = webdriver.DesiredCapabilities.CHROME.copy()
        options = cls._get_extensions_options(config_chrome_section)
        # remote or local chrome
        if config_chrome_section["remote"]:
            remote_capabilities = dict(capabilities.items() + options.to_capabilities().items())
            uri = config_chrome_section["uri"]
            browser = webdriver.chrome.webdriver.RemoteWebDriver(command_executor=uri,
                                                                 desired_capabilities=remote_capabilities)
            print("Remote webdriver connecting to %s") % uri
        else:
            driver = config_chrome_section["chrome_driver"]["path"]
            # Set environment variable for Chrome.
            # Chrome driver needs to have an environment variable set,
            # this must be set to the path to the webdriver file.
            os.environ["webdriver.chrome.driver"] = driver
            browser = webdriver.Chrome(executable_path=driver,
                                       chrome_options=options,
                                       desired_capabilities=capabilities)
        return browser

    def setup_method(self, method):
        """
        Base method called before every test case method is called.

        """
        self.config = self.get_config()
        self.current_method = method.__name__
        print "current test: %s" % self.current_method

    def teardown_method(self, _):
        """
        Base method called always after the test case method was
        performed regardless of its result.

        """
        self.browser.quit()

    def get_camera_pose(self, browser=None):
        """
        Return current camera pose object by calling the acme object.
        It either takes default instance member browser reference
        or the one provided by the argument

        Kwargs:
            options browser reference

        """
        curr_browser = browser if browser else self.browser
        # simple call 'return acme.getCameraPose();' return shallow copy,
        # methods (keys) like getAlt do exist but point to empty dicts
        # once they made it to python from javascript
        #res = curr_browser.execute_script('return acme.getCameraPose();')
        js = ("return [acme.getCameraPose().getAlt(), "
                      "acme.getCameraPose().getLon(), "
                      "acme.getCameraPose().getLat()];")
        res = curr_browser.execute_script(js)
        p = Pose(res[0], res[1], res[2])
        return p

    @staticmethod
    def pose_is_near(left,
                     right,
                     alt_delta=1,
                     lon_delta=0.000001,
                     lat_delta=0.000001,
                     assert_alt=True,
                     assert_lon=True,
                     assert_lat=True):
        """
        Checks if one of the pose objects is near to another.

        We need this function, as sometimes the maps camera moves a little bit,
        however users still see the same place. It is caused by two factors:
        - internal behaviour of the maps
        - floating numbers behaviour

        Args:
            left                 - the first pose object to compare
            right                - the second object to compare
            alt_delta = 1        - the acceptable difference of alt attribute
            lon_delta = 0.000001 - the acceptable difference of lon attribute
            lat_delta = 0.000001 - the acceptable difference of lat attribute
            assert_alt = True    - should the altitude be asserted
            assert_lon = True    - should the longitude be asserted
            assert_lat = True    - should the latitude be asserted

        Returns:
            boolean based on the comparison result

        """
        if assert_alt:
            alt = abs(left.alt - right.alt) < alt_delta
        else:
            alt = True
        if assert_lon:
            lon = abs(left.lon - right.lon) < lon_delta
        else:
            lon = True
        if assert_lat:
            lat = abs(left.lat - right.lat) < lat_delta
        else:
            lat = True
        return alt and lon and lat

    def click(self, finder_value, finder):
        """
        Performs javascript click on the element.

        Args:
            finder_value: string to find (class name or id according to finder kind)
            finder: 'by_class' or 'by_id'

        Usage:
            self.click("searchbutton", finder="by_class")
            self.click("acme-famous-places-button", finder="by_id")

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
        elif finder == "by_id":
            finder_script = """
            var element = document.getElementById('{0}');
            if (element == null) {{
                alert('Didn\\'t find the element with ID = {0}');
                return;
            }}
            """.format(finder_value)
        else:
            raise ValueError("Wrong finder value: '%s'" % finder)
        event_script = "(function(){{ {0} ; element.click(); }}());".format(finder_script)
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

        TODO:
        this method is not needed anymore, remove?

        """
        return self._get_zoom_level_from_url(self.browser.current_url)

    def _get_zoom_level_from_url(self, url):
        """
        Returns zoom level from the given url.

        Sample urls, chrome switches from one to another from time to time:
            https://www.google.com/maps/@8.135687,-75.0973243,17856994a,40.4y,1.23h/data=!3m1!1e3
            https://www.google.com/maps/@8.135687,-75.0973243,18207688m/data=!3m1!1e3

        Argus:
            url taken from the browser on maps.google.com

        Returns:
            (int) zoom level read from the url

        TODO:
        this method is not needed anymore, remove?

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

        Args:
            old_value - old value for the url, checked again the current url
            interval  - sleep time between checks
            max_wait_time - maximum time for the function to run

        """
        start = time.time()
        while (start + max_wait_time > time.time()):
            #print self.browser.current_url
            if old_value != self.browser.current_url:
                return
            time.sleep(interval)
