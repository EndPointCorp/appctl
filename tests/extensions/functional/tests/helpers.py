"""
Helpers used by tests
"""

from exception import HelperException
from functools import partial

from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as exp_cond

from base import TestBase


def filter_list_of_dicts(lyst, key, value):
    """
    filter dict from list of two value dicts - helpful for javascript objects
    when you've got "X" and "Y"
    [{"Description": "X", "Value": "Y"},{...}
    """
    filtered_list = filter(lambda x: key in x.values() and value in x.values(), lyst)

    if len(filtered_list) == 1:
        return True
    else:
        raise HelperException("Error filtering list")



def wait_for_loaded_page(url,
                         browser,
                         elem_identifier_kind=By.CLASS_NAME,
                         elem_identifier_name="widget-compass"):
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

    returns:
        Nothing

    """
    config = TestBase.get_config()
    browser.get(url)
    # wait for compass to appear, then start testing by taking a screenshot
    tester = partial(exp_cond.visibility_of_element_located,
                     (elem_identifier_kind, elem_identifier_name))
    msg = "Element identified by '%s' did not appear." % elem_identifier_name
    WebDriverWait(browser,
                  config["max_load_timeout"]).until(tester(), message=msg)
