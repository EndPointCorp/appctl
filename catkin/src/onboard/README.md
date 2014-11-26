onboard
-------

### listener.py

A ROS node for showing or hiding an on-screen keyboard.  The backend is `dbus` calls and configuration is loaded from `$HOME/etc/onboard.dconf`.

##### Topics

* `/onboard/visibility` : `std_msgs/Bool` - Shows the keyboard when this topic hears `True`, hides it on `False`.
