"""
Show chrome with extension loaded.
Useful for DOM HTML tree elements inspection with
    chrome extension loaded, like when running the tests.

"""

from tests.base import TestBase

klass = TestBase
klass.setup_class()  # sets the configuration
config = klass.get_config()
#browser = klass.run_browser(config["chromes"]["kiosk"])
browser = klass.run_browser(config["chromes"]["display"])
browser.get(config["maps_url"])
