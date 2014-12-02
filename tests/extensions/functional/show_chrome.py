"""
Show chrome with extension loaded.
Useful for DOM HTML tree elements inspection with
    chrome extension loaded, like when running the tests.

"""

from tests.base import TestBase
from tests.base import TestBaseTouchscreen
from tests.base import TestBaseGeneric


klass = TestBaseGeneric
klass.extensions = ["kiosk"]
klass.setup_class()  # reading of the configuration happens here
config = klass.get_config()
browser = klass.run_browser()
browser.get(config["maps_url"])
