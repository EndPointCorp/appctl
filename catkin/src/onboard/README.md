onboard
-------

### description

Current implementation is able to show the onboard on both nodes depending on the configuration.

In the `attended` mode the onboard is shown only on the 42-a.

In the `tactile` mode the onboard is shown only on the 42-b.

On both nodes we have have the `onboard/listener.py` and `onboard/launcher.py`. The listener converts the ROS show/hide messages to dbus ones. The launcher launches/kills the onboard process on the display nodes depending on the mode. We should have only one onboard running.

This way on the dbus messages are run on both display nodes, but there is only one onboard process which listens to them.

#### showing keyboard in attended mode

The Chrome Onboard extension adds an icon next to the url field. Tapping it sends a ROS message to show/hide the onboard. This is usable in the attended mode. The best solution would be to change the Chrome behaviour to work with the Onboard directly, but for now the icon is the best I could think about.

#### display/listener.py

On each display node (42-a and 42-b) there should be running the `onboard/listener.py` ROS node.

This should be done using the chef node definition file.

Just remember that all the nodes should be named differently, so we cannot have two 'onboard' nodes. I also noticed that using a dash in the ros node name doesn't work, something removes the dash and everything which is after it. So the names like `onboard-42a` and `onboard-42b` will be treated like the same name `onboard`. Using the same name will make a race condition, the first ros node will run, the second will be restarting filling logs with something like `there is a node with the same name`. The names like `onboard_42a` work.

#### onboard/launcher.py

On each display node we also need the onboard/launcher.py. Also with different ROS node names. In the chef node definition file there should be the name of the mode the onboard should run on each node. The params should be like: `"params": [{ "name": "modes",    "value": "attended"}`.

#### chrome onboard extension

On each browser (kiosk, display) we need to have the same onboard extension loaded.
This package contains ROS nodes for launching and showing/hiding onboard keyboard. The `appctl` support library is used for mode activation. 

### listener.py

A ROS node for showing or hiding an on-screen keyboard.  The backend is `dbus` calls and configuration is loaded from `$HOME/etc/onboard.dconf`.

##### Topics

* `/onboard/visibility` : `std_msgs/Bool` - Shows the keyboard when this topic hears `True`, hides it on `False`.

### launcher.py

A ROS node for running onboard app in configured mode. It uses `apctl` for mode activation. The onboard is run hidden, and should be shown/hidden using the ROS messages supported by the `listener.py`.

##### Parameters

* `modes` : Comma-separated list of modes to run this browser in, required.

