"""
Show chrome browsers with extensions loaded.
Useful for DOM HTML tree elements inspection with
    chrome extension loaded, like when running the tests.

ROS communication happens between browsers,
    kiosk browser drives the display browser.

roslaunch catkin/src/portal/launch/portal.launch
    need to run first

python -i show_ros_chromes.py

"""

from tests.base import TestBase


klass = TestBase
# reading of the configuration happens here, necessary just once
klass.setup_class()
config = klass.get_config()

browser_kiosk = klass.run_browser(config["chromes"]["kiosk"])
browser_kiosk.get(config["maps_url"])
print "chrome browser kiosk up ..."

browser_display = klass.run_browser(config["chromes"]["display"])
browser_display.get(config["maps_url"])
print "chrome browser display up ..."