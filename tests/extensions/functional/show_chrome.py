"""
Show chrome with extension loaded.
Useful for DOM HTML tree elements inspection with
    chrome extension loaded, like when running the tests.

"""

import time

from selenium import webdriver

from tests.base import TestBaseTouchscreen
from tests.base import TestBase
from tests.base import MAPS_URL

#klass = TestBaseTouchscreen
klass = TestBase
klass.extensions = ["kiosk"]
klass.setup_class()
# if browser automatically redirects to national mutation,
# our chrome extensions are loaded, yet do not work
# redirection doesn't happen with MAPS_URL
browser = klass.run_browser()
print "Browser capabilities:\n%s" % browser.capabilities
browser.get(MAPS_URL)
