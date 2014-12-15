"""
Helpers used by tests.

"""


import os
from functools import partial
from functools import wraps
import traceback
import datetime

from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as exp_cond

from exception import HelperException
from base import TestBase


def filter_list_of_dicts(lyst, key, value):
    """
    filter dict from list of two value dicts - helpful for javascript objects
    when you've got "X" and "Y"
    [{"Description": "X", "Value": "Y"},{...}]

    """
    filtered_list = filter(lambda x: key in x.values() and value in x.values(), lyst)

    if len(filtered_list) == 1:
        return True
    elif len(filtered_list) == 0:
        return False
    else:
        raise HelperException("Error filtering list")


def wait_for_loaded_page(url,
                         browser,
                         elem_identifier_kind=By.CLASS_NAME,
                         elem_identifier_name="widget-compass",
                         elem_presence=True):
    """
    Waits for a page to load URL.
    Now it seems to suffice to wait until 'widget-compass' element appears.
    This is default on pages with compass widget loaded.

    Waits specified pause and time outs if the element does not appear.

    browser: web browser instance
    elem_identifier_kind: kind of element identification, can be
    either By.CLASS_NAME or By.ID

    elem_identifier_name: particular element class name or ID (according
    to which is chosen by the elem_identifier_kind argument.

    elem_presence flips the condition, so when False we check element's
        non presence

    """
    config = TestBase.get_config()
    browser.get(url)
    # wait for compass to appear, then start testing by taking a screenshot
    tester = partial(exp_cond.visibility_of_element_located,
                     (elem_identifier_kind, elem_identifier_name))
    if elem_presence:
        msg = "Element identified by '%s' did not appear within timeout." % elem_identifier_name
        WebDriverWait(browser,
                      config["max_load_timeout"]).until(tester(), message=msg)
    else:
        msg = "Element identified by '%s' still present after timeout." % elem_identifier_name
        WebDriverWait(browser,
                      config["max_load_timeout"]).until_not(tester(), message=msg)


def make_screenshot(browser, test_name, index):
    """
    Construct file name for the screen shot and grap the screen via selenium.

    """
    config = TestBase.get_config()
    ss_dir = config["screenshots_dir"]
    if not os.path.exists(ss_dir):
        os.makedirs(ss_dir)
    fname = "{}{}{}_{}_{}".format(ss_dir,
                                  os.path.sep,
                                  datetime.datetime.now().strftime("%Y-%m-%d--%H-%M-%S"),
                                  test_name,
                                  index)
    print "Making screenshot: " + fname
    browser.save_screenshot("{}.png".format(fname))
    return fname


def screenshot_on_error(test):
    """
    Decorator for test functions to make screenshots on error.

    The wrapper runs the test. When it fails, then it makes
    a screenshot in the config "screenshots_dir" directory.

    """
    @wraps(test)
    def wrapper(*args, **kwargs):
        try:
            test(*args, **kwargs)
        except:
            test_obj = args[0]
            test_name = test_obj.current_method

            fname = ""
            js_console = dict()
            if hasattr(test_obj, "browsers"):
                # is multiple browsers test
                for ext_name, browser in test_obj.browsers.items():
                    fname = make_screenshot(browser, "%s-%s" % (test_name, ext_name), 0)
                    # grab javascript console output too
                    # seems to catch only error states from javasript
                    # doing arbitrary console.log() and reading it back
                    # via this call returns empty list
                    # also, repeated browser.get_log calls eventually return
                    # empty list as the opportune previous error messages were
                    # consumed (?) by the .get_log() call?
                    js_console[ext_name] = browser.get_log("browser")
            else:
                # is a single browser test
                browser = test_obj.browser
                fname = make_screenshot(browser, test_name, 0)

            with open("{}.log".format(fname), "w") as flog:
                flog.write(traceback.format_exc())
                flog.write("\n\njavascript console output:\n\n")
                for extention, entries in js_console.items():
                    flog.write("%s:\n" % extention)
                    flog.write("\n".join([str(entry) for entry in entries]))
                    flog.write("\n\n\n")
            raise
    return wrapper
