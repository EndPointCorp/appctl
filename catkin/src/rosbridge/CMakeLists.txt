cmake_minimum_required(VERSION 2.8.3)
project(rosbridge)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosbridge
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



