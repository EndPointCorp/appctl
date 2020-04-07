#!/bin/bash

set -e

cd /catkin_ws
source ./devel/setup.bash
pycodestyle --count src
catkin_lint src
catkin_make run_tests_${THIS_PROJECT} -DNOSETESTS=/usr/bin/nosetests3
catkin_test_results
