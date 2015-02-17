onboard
-------

This package contains ROS nodes for launching and showing/hiding onboard keyboard. The `appctl` support library is used for mode activation. 

### listener.py

A ROS node for showing or hiding an on-screen keyboard.  The backend is `dbus` calls and configuration is loaded from `$HOME/etc/onboard.dconf`.

##### Topics

* `/onboard/visibility` : `std_msgs/Bool` - Shows the keyboard when this topic hears `True`, hides it on `False`.

### launcher.py

A ROS node for running onboard app in configured mode. It uses `apctl` for mode activation. The onboard is run hidden, and should be shown/hidden using the ROS messages supported by the `listener.py`.

##### Parameters

* `modes` : Comma-separated list of modes to run this browser in, required.

