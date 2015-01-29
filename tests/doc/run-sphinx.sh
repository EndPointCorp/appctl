#!/bin/bash -ex

# this is necessary since sphinx imports modules
# including test_ros which depends on libraries from inside catkin
cd ../../
source catkin/devel/setup.bash
cd -

make clean
make singlehtml
make latexpdf
