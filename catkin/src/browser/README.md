Browser
=======

This package contains a ROS node for launching a browser.

### launcher.py

Launches a browser during specified appctl modes.  Presently, this is a wrapper around lg-browser.

##### Parameters

* `modes` : Comma-separated list of modes to run this browser in, required.

* `url` : The url to open, required.

* `window` : The window name, defaults to lgS0.

* `app` : The app name, defaults to appctl.

* `disable_extensions` : If true, disable all browser extensions.

* `disable_kiosk` : If true, disable kiosk mode.

* `debug_port` : Use a specific debug port.  This is normally auto-detected ephemerally in lg-browser.

* `browser_bin` : The binary to use.  lg-browser controls the default.

* `scale_factor` : Scale the browser content by this factor.

* `user_agent` : Spoof a UserAgent.

* `extensions` : Comma-separated list of paths to unpacked extensions to be loaded.

### monitor.py

Launches a browser monitor during specified appctl modes.

##### Parameters

* `modes` : Comma-separated list of modes to run the monitor in

* `window` : The window name, defaults to lgS0.

* `app` : The app name, defaults to appctl.

* `debug_port` : Use a specific debug port.  This is normally auto-detected ephemerally in lg-browser.

* `names` : Comma-separated list of names to look for. If not specified, the list is pulled from /lg/personavars.txt
