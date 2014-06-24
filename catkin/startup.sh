#!/bin/bash
source ./devel/setup.bash
roscore &
sleep 3
roslaunch rosbridge_server rosbridge_websocket.launch &
echo "starting spacenav_node"
rosrun spacenav_node spacenav_node &
echo "starting slinky_nav"
rosrun slinky_nav slinky_nav
echo "exiting"
exit 0

