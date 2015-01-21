portal\_leds
------------

ROS software for interactions between LED strip control boards and the rest of the Portal stack.

### controller.py

Determines which messages to send to the LED control board and when.

The current implementation subscribes to occupancy messages and sends 'hello' and 'goodbye' commands.

##### Topics

* `/portal_leds/command` : `std_msgs/String` - Commands to be written to the LED control board.

### writer.py

Writes commands to LED control board via serial interface.  Listens on `/portal_leds/command`.

##### Parameters

* `device_path` : Path to the control board's serial interface.

* `baud_rate` : Baud rate for serial communications.  Default `9600`.
