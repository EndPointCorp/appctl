FROM ros:indigo
MAINTAINER Matt Vollrath <matt@endpoint.com>
ARG BUILD_DEBS='false'

# install system dependencies
RUN apt-get update \
 && apt-get install -y \
      git \
      g++ \
      pep8 \
      python-catkin-lint \
 && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS /catkin_ws
ENV DEP_WS /dep_ws

RUN mkdir -p $CATKIN_WS/src \
 && . /opt/ros/indigo/setup.sh \
 && catkin_init_workspace $CATKIN_WS/src \
 && echo '#!/bin/bash' > /ros_entrypoint.sh \
 && echo 'source $CATKIN_WS/devel/setup.bash' >> /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

# Install lg-ros-build, only if we are building debs
RUN if [ "$BUILD_DEBS" = "true" ]; then \
 . /opt/ros/indigo/setup.sh \
 && mkdir -p $DEP_WS/src \
 && catkin_init_workspace $DEP_WS/src \
 && cd $DEP_WS \
 && git clone https://github.com/EndPointCorp/lg_ros_nodes --depth 1 /tmp/lg_ros_nodes \
 && ln -snf /tmp/lg_ros_nodes/lg_builder $DEP_WS/src/ \
 && apt-get update \
 && apt-get -y install debhelper \
 && rosdep update \
 && rosdep install -y --from-paths $DEP_WS/src --ignore-src --rosdistro=indigo \
 && catkin_make \
 && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo \
 && rm -rf /tmp/lg_ros_nodes \
 && rm -rf $DEP_WS \
 && mkdir /output \
 && rm -rf /var/lib/apt/lists/* \
 && rm -rf ~/.ros \
;fi

WORKDIR $CATKIN_WS
COPY appctl $CATKIN_WS/src/appctl
COPY setup.cfg $CATKIN_WS/setup.cfg
RUN . /opt/ros/indigo/setup.sh \
 && catkin_make
