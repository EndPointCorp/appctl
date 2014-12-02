maxbotix
--------

ROS node for reading a Maxbotix proximity sensor.

### sender.py

Attaches to a proximity sensor via USB serial port and publishes proximity information.

##### Topics

* `/proximity/distance` : `sensor_msgs/Range` - Distance (in meters) to nearest object in the sensor's field of view.

* `/proximity/presence` : `std_msgs/Bool` - Reports true if the sensor is reporting an object is in the field of view.  This is a "normalized" and somewhat delayed simplification for cases where we want to know if an object is present, but the theshold range is set by the device.
