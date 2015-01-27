rfreceiver
----------

A set of ROS tools for an Arduino-based RF receiver and remote keyfob.

### sender.py

A node that attaches to a promicro board serially and publishes button pushes.

##### Parameters

* `device_path` : Required.  Path to the serial device.  Remember that most devices will be available in `/dev/serial/by-id`.

* `baud_rate` : Baud rate for serial communications.  Defaults to 9600.

##### Topics

* `/rfreceiver/buttondown` : `std_msgs/Byte` - A number indicating the button being pushed, where "A" is "1" and so on.

### kill\_browser.py

A node that listens for a button press and kills all browser instances if it is the "B" button.

This is mostly hard-coded for Campfire.
