rfreceiver
----------

A set of ROS tools for an Arduino-based RF receiver and remote keyfob.

### sender.py

A node that attaches to a promicro board serially and publishes button pushes.

##### Topics

* `/rfreceiver/buttondown` : `std_msgs/Byte` - A number indicating the button being pushed, where "A" is "1" and so on.

### kill\_browser.py

A node that listens for a button press and kills all browser instances if it is the "B" button.

This is mostly hard-coded for Campfire.
