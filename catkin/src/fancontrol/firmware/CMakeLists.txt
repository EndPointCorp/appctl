cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(code SKETCH controller.cpp SRCS ${ROS_LIB_DIR}/time.cpp BOARD uno PORT /dev/ttyACM0)
