cmake_minimum_required(VERSION 2.8.3)
project(appctl_msg_defs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_message_files(
  FILES
  Mode.msg
)

add_service_files(
  FILES
  Query.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
