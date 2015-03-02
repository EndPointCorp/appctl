connectivity
------------

### description

A ROS node that detects internet problems. When there's no internet it
will emit a message telling portal to go offline (e.g. display an
offline video)

To emulate offline mode just touch a `/tmp/test_offline_mode` file

### watchman.py

##### Topics

* `/connectivity/online` : `std_msgs/Bool` - should return true if
  there's internetz, and false if there's no internetz

* `/appctl/mode` : `appctl/Mode` - publishes an "offline_video" mode
  after detecting no internetz

##### Services

* `/connectivity/online` : `std_msgs/Bool` - same as for topic but as a
  service

##### Parameters

* `max_failed_attempts` : integer defining retry attempts to some
  internet sites before deciding on going offline - default = `5`
* `timeout` : how long to wait for http HEAD response - deafult = `1`
  second
* `offline_mode_name`: the mode name that should be set after detecting
  problems with internet
* `online_mode_name`: mode name that should be enabled after internet is
  recovered
