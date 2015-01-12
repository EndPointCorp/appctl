statistics
----------

ROS bridge for statsd and google retail store statistics provider

### Message Types

* `statistics/StatsD` - A single statsd metric report.

## Scripts

### listener.py

A node the listens for statistics messages, converts them to statsd strings, and sends the datagrams to the statsd server.

##### Parameters

* `statsd_host` - The host on which the target statsd server is listening.  Default `lg-head`.

* `statsd_port` - The port on which the target statsd server is listening.  Default `8125`.

##### Topics

`/statistics/statsd` : `statistics/StatsD` - Incoming statistics metrics.

### session_aggregator.py

A node that provides a ros node and ros service.

 - ros node listens for session events on /statistics/session and stores them safely
 - ros service provides ros service for retrieval of aggregated
   sessions. There's a flag that can be set during query to erase all
   aggregated sessions

##### Incoming session event data structure

```json
{
  'start_ts' : 0,
  'end_ts': 0,
  'conversion' : true | false,
  'conversion_num' : 0,
  'app_name' : doodle | sth | tactile | pacman
}
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

### file_writer.py

A ros node that aggregates session_aggregator sessions and formats them
for writing into json file on the local filesystem.

##### Topics

None

##### Services

Queries `/statistics/session`

##### Parameters

* `aggregation_interval` - interval in seconds between which events
  should be collected - defaults to 300 seconds
* `json_output_path` - local path to where should the json files be
  written to. Defaults to /tmp/
