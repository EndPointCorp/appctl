Geodata Service
===============

This service returns population counts for queries consisting of a latitude,
longitude, and radius in kilometers(?).

The data file comes from:

<http://sedac.ciesin.columbia.edu/data/set/gpw-v3-population-count-future-estimates/data-download>

    Population Count Grid Future Estimates, v3 (2015)

### export\_data.py

This script takes the raw .asc data and turns it into a compressed numpy array.
This compressed numpy array should be distributed with the service.

##### params

* ~src - Path to the input file (default: /tmp/glp15ag.asc).

* ~dst - Path of the output file (default: /tmp/geodata\_population.npz).

### geodata\_server.py

Runs the geodata service.

##### services

* /geodata/population : GeodataQuery.srv

##### params

* ~src - Path to the data file (default:
  /opt/ros/$ROS\_DISTRO$/share/geodata/geodata\_population.npz).

