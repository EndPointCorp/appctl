"""
Show chrome with extension loaded.
Useful for DOM HTML tree elements inspection with
    chrome extension loaded, like when running the tests.

"""

from tests.base import MAPS_URL


from tests.base import TestBase
from tests.base import TestBaseTouchscreen
from tests.base import TestBaseGeneric


klass = TestBaseGeneric
klass.extensions = ["kiosk"]
klass.setup_class()  # reading of the configuration happens here
browser = klass.run_browser()
browser.get(MAPS_URL)
