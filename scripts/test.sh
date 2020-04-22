#!/bin/bash

set -e

cd /catkin_ws
source ./devel/setup.bash
pycodestyle --count src
catkin_lint src
catkin_make run_tests_appctl -DNOSETESTS=/usr/bin/nosetests3
catkin_test_results
