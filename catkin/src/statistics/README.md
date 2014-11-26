statistics
----------

ROS bridge for statsd.

### Message Types

* `statistics/StatsD` - A single statsd metric report.

### listener.py

A node the listens for statistics messages, converts them to statsd strings, and sends the datagrams to the statsd server.

##### Parameters

* `statsd_host` - The host on which the target statsd server is listening.  Default `lg-head`.

* `statsd_port` - The port on which the target statsd server is listening.  Default `8125`.

##### Topics

`/statistics/statsd` : `statistics/StatsD` - Incoming statistics metrics.
