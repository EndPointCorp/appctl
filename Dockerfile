FROM ros:melodic
MAINTAINER Matt Vollrath <matt@endpoint.com>
ARG BUILD_DEBS='false'

# install system dependencies
RUN apt-get update \
 && apt-get install -y \
      git \
      g++ \
      pep8 \
      python-catkin-lint \
      python3-pip \
      python3-nose \
 && pip3 install setuptools \
 && pip3 install wheel pyyaml rospkg \
 && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS /catkin_ws
ENV DEP_WS /dep_ws

RUN mkdir -p $CATKIN_WS/src \
 && . /opt/ros/melodic/setup.sh \
 && catkin_init_workspace $CATKIN_WS/src \
 && echo '#!/bin/bash' > /ros_entrypoint.sh \
 && echo 'source $CATKIN_WS/devel/setup.bash' >> /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

# Install lg-ros-build, only if we are building debs
RUN if [ "$BUILD_DEBS" = "true" ]; then \
 . /opt/ros/melodic/setup.sh \
 && mkdir -p $DEP_WS/src \
 && catkin_init_workspace $DEP_WS/src \
 && cd $DEP_WS \
 && git clone https://github.com/EndPointCorp/lg_ros_nodes --depth 1 /tmp/lg_ros_nodes \
 && ln -snf /tmp/lg_ros_nodes/lg_builder $DEP_WS/src/ \
 && apt-get update \
 && apt-get -y install debhelper \
 && rosdep update \
 && rosdep install -y --from-paths $DEP_WS/src --ignore-src --rosdistro=melodic \
 && catkin_make \
 && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic \
 && rm -rf /tmp/lg_ros_nodes \
 && rm -rf $DEP_WS \
 && mkdir /output \
 && rm -rf /var/lib/apt/lists/* \
 && rm -rf ~/.ros \
;fi

WORKDIR $CATKIN_WS
COPY appctl $CATKIN_WS/src/appctl
COPY setup.cfg $CATKIN_WS/setup.cfg
COPY appctl_msg_defs $CATKIN_WS/src/appctl_msg_defs
RUN . /opt/ros/melodic/setup.sh \
 && catkin_make
