evdev\_teleport
---------------

#### Cross-device input device replication

This package contains tools for replaying input events on remote systems.  For
example, you could publish the input stream of a mouse or joystick and replay
the events on one or many other systems as virtual devices.  These input events
are intercepted and replayed with kernel interfaces, making it higher level but
functionally nearly equivalent to a USB extender without the wiring
requirement.

### Usage

##### sender\_node

The path to the real device is set by the private parameter ~device\_file.
i.e.

    rosrun evdev_teleport sender_node _device_file:=/dev/input/event2

##### receiver\_node

The name of the virtual device is set by the private parameter ~device\_name.
i.e.

    rosrun evdev_teleport receiver_node _device_name:="Virtual Thing"

Receivers have a special activation topic at
`/evdev_teleport/activation/<node_name>`.  The `evdev_teleport_switcher.py`
script in the `appctl` package can be used to make the receiver mode-aware.

##### TODO / Limitations

Receivers use hard-coded device capabilities for an ELO touchscreen.  Ideally
the sender should provide a service for querying the capabilities of the
teleported device.  An intermediate step would be to parameterize the absolute
min/max axis values and max touches.

### Reference

[Explanation of event types and codes][0]

[Event types and codes (input.h)][1]

[Input interface][2]

[Uinput interface][3]

[0]: https://www.kernel.org/doc/Documentation/input/event-codes.txt
[1]: http://lxr.free-electrons.com/source/include/uapi/linux/input.h
[2]: http://lxr.free-electrons.com/source/include/linux/input.h
[3]: http://lxr.free-electrons.com/source/include/linux/uinput.h
