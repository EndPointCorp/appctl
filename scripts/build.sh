#!/usr/bin/env bash

set -e
set -x

cd /catkin_ws
source devel/setup.bash
./pack-debs
