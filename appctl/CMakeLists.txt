cmake_minimum_required(VERSION 2.8.3)
project(appctl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  appctl_msg_defs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs appctl_msg_defs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/controller.py
  scripts/evdev_teleport_switcher.py
  scripts/rfreceiver_mode_select.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/appctl_support/test_mode_handler.py)
  catkin_add_nosetests(test/appctl_support/test_proc_controller.py)
  catkin_add_nosetests(test/appctl_support/test_proc_runner.py)
  add_rostest(test/mode_spin.test)
endif()
