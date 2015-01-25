portal\_occupancy
-----------------

ROS software for determining Portal occupancy based on aggregation of sensor messages.

### aggregate.py

A node that listens to sensors and publishes occupancy state.

* Proximity sensor via `/proximity/distance`
* SpaceNav via `/spacenav/twist`
* LEAP (does not engage occupancy) via `/leap_motion/frame`
* Touchscreen via `/evdev_teleport/events`

##### Topics

* `/portal_occupancy/occupancy/is_active` : `std_msgs/Bool` - True if the Portal is occupied, False if it is not.

* `/portal_occupancy/occupancy/inactive_duration` : `std_msgs/Duration` - How long the Portal has been unoccupied (or zero).

* `/portal_occupancy/interaction/is_active` : `std_msgs/Bool` - True if a user is interacting with the Portal, False if not.

* `/portal_occupancy/interaction/inactive_duration` : `std_msgs/Duration` - How long since the last interactive session ended (or zero).

"Occupancy" is active when a user is standing at the podium and/or interacting with the system.

"Interaction" is activated when the user touches an input peripheral and deactivated when proximity and input are no longer detected.

##### Parameters

* `check_interval` : How many seconds to wait between checks.

* `distance_threshold` : Distance in meters at which the proximity sensor detects occupancy.
