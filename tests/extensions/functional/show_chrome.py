"""
Show chrome with extension loaded.
Useful for DOM HTML tree elements inspection with
    chrome extension loaded, like when running the tests.

"""

from tests.base import TestBase

klass = TestBase
klass.setup_class()  # sets the configuration
config = klass.get_config()
# 3 - the number of the chrome configuration within the "chromes" section
browser = klass.run_browser(config["chromes"]["kiosk"])
browser.get(config["maps_url"])
