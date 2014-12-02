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

from tests.base import TestBaseGeneric


klass_kiosk = TestBaseGeneric
klass_kiosk.extensions = ["kiosk"]
# reading of the configuration happens here, necessary just once
klass_kiosk.setup_class()
config = klass_kiosk.get_config()
browser_kiosk = klass_kiosk.run_browser()
browser_kiosk.get(config["maps_url"])
print "chrome browser kiosk up ..."

klass_display = TestBaseGeneric
klass_display.extensions = ["display"]
browser_display = klass_display.run_browser()
browser_display.get(config["maps_url"])
print "chrome browser display up ..."