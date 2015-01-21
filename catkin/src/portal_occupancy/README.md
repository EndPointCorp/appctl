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

* `/portal_occupancy/state` : `std_msgs/Bool` - True if the Portal is occupied, False if it is not.  There is no "grace period" for this reading, it is the responsibility of the listener to set timeouts.

* TODO: `/portal_occupancy/idle` : `std_msgs/Duration` - How long the Portal has been unoccupied, or zero if occupied.

##### Parameters

* `check_interval` : How many seconds to wait between checks.

* `distance_threshold` : Distance in meters at which the proximity sensor detects occupancy.
