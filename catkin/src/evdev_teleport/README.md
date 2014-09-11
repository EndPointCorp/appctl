evdev\_teleport
===============

#### Cross-device input device replication

This package contains tools for replaying input events on remote systems.  For
example, you could publish the input stream of a mouse or joystick and replay
the events on one or many other systems as virtual devices.  These input events
are intercepted and replayed with kernel interfaces, making it higher level but
functionally nearly equivalent to a USB extender without the wiring
requirement.

### Usage

##### sender\_node

The path to the device is set by the private parameter ~device\_file.  i.e.

    rosrun evdev_teleport sender_node _device_file:=/dev/input/event2

