statistics
----------

ROS bridge for statsd and google retail store statistics provider

### Message Types

* `statistics/StatsD` - A single statsd metric report.

## Scripts

### session\_tracker.py

A node that listens for portal_occupancy "occupied" messages and breaks
the current session. It caches current session so when portal becomes
occupied again it emits Session start with mode and application that was
used before portal went to ambient mode.

Requires portal\_occupancy::aggregate.py to operate effectively.

##### Topics

* Subscribes to `/portal_occupancy/interaction/inactive_duration`
* Publishes to `/statistics/session`

##### Parameters

* `inactivity_timeout` - How long to wait before declaring a session over,
  in seconds.  Default: 20.0

### statsd_listener.py

A node the listens for statistics messages, converts them to statsd strings, and sends the datagrams to the statsd server.

##### Parameters

* `statsd_host` - The host on which the target statsd server is listening.  Default `lg-head`.

* `statsd_port` - The port on which the target statsd server is listening.  Default `8125`.

##### Topics

`/statistics/statsd` : `statistics/StatsD` - Incoming statistics metrics.

### session\_aggregator.py

A node that provides a ros node and ros service.

 - ros node listens for session events on /statistics/session and stores them safely
 - ros service provides ros service for retrieval of aggregated
   sessions. There's a flag that can be set during query to erase all
   aggregated sessions


##### Incoming session event data structure

Each Session emitter should send following event:

```
start_ts: 1421245409
end_ts: 1421245430
application: 'pacman'
mode: 'tactile'
prox_sensor_triggered: True
```

##### Parameters

* `max_events` - maximum number of stored events. Default - infinite.
  When this treshold is reached, old events will be discarded upon each
  new event arrival.
* `max_memory` - maximum size of storage object in bytes. Default - 32000000
  Above this, old session events will be discarded and alog message will be
  produced

##### Topics

Provides `/statistics/session` : `statistics/session` - Incoming sessions

##### Services

Provides `/statistics/session` : `statistics/SessionQuery` - Querying aggregator
with `erase` flag that discards the events that got read.


### file\_writer.py

A ros node that aggregates session\_aggregator sessions and formats them
for writing into json file on the local filesystem. It writes glink json
files in glink format and in extended format for End Point

##### Topics

None

##### Services

Queries `/statistics/session`

##### Parameters

* `aggregation_interval` - interval in seconds between which events
  should be collected - defaults to 300 seconds
* `json_output_path` - local path to where should the json files be
  written to. Defaults to /tmp/

### Tests

Run the tests with `catkin_make`:

    $ catkin_make test

Or with `rostest`:

    $ rostest statistics test_statistics.test

#### Testing scenario for statistics

Possible edge cases:
- change /appctl/mode few times
- try to enter pacman and change mode
- enter "tactile" and go away for 30 secs so portal goes to ambient mode
  and then come back
- do above but dont come back - restart roslaunch on displaynodes
  instead

Example procedure:

- After launch there should be a started session.

```shell
rosservice call /statistics/session "erase: true
current_only: true"
```

- Straight after launch, a session should be cached and continued after
  a prox_sensor break.

```shell
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 30
  nsecs: 1"
rosservice call /statistics/session "erase: false
current_only: true"
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 1
  nsecs: 0"
rosservice call /statistics/session "erase: false
current_only: true"
```

- Let's emulate following scenario:
 - launch portal
 - emulate entering ambient mode (no one near the touchscreen)
```shell
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 30
  nsecs: 0"
```
 - emulate breaking ambient mode (someone has come near portal)
```shell
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 1
  nsecs: 0"
```
 - emulate using pacman app
```shell
rostopic pub /statistics/session statistics/Session "{mode: '', application: 'pacman', start_ts: `date +%s`, end_ts: 0, prox_sensor_triggered: false}"
```
 - emulate entering ambient mode (someone left from playing pacman)
```shell
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 30
  nsecs: 0"
```
 - emulate breaking ambient mode (someone has came)
```shell
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 1
  nsecs: 0"
```
 - emulate entering attended mode
```shell
rostopic pub /appctl/mode appctl/Mode "mode: 'attended'"
```
 - emulate entering ambient mode (someone left from playing pacman)
```shell
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 30
  nsecs: 0"
```
 - emulate breaking ambient mode (someone has came)
```shell
rostopic pub /portal_occupancy/interaction/inactive_duration std_msgs/Duration "data:
  secs: 1
  nsecs: 0"
```

Number of sessions:
 - tactile session
 - another tactile session
 - pacman session
