"""
Show chrome with extension loaded.
Useful for DOM HTML tree elements inspection with
    chrome extension loaded, like when running the tests.

"""

from tests.base import MAPS_URL


from tests.base import TestBase
from tests.base import TestBaseTouchscreen
from tests.base import TestBaseROS


# - read config
# - based on the config 'chrome' attrib spawn proper browsers

#klass = TestBaseTouchscreen
#klass.setup_class()
#browser = klass.run_browser()
#print "Browser capabilities:\n%s" % browser.capabilities
#browser.get(MAPS_URL)
