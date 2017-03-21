FROM jeguzzi/ros:kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y --fix-missing\
   ros-kinetic-robot-state-publisher \
   ros-kinetic-roslint \
   ros-kinetic-compressed-image-transport \
   ros-kinetic-image-transport-plugins \
   sudo \
   && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy

RUN apt-get update && apt-get install -y --fix-missing\
   python-rosdep

RUN /bin/bash -c 'rosdep update'
RUN /bin/bash -c 'rosdep install --from-paths src -i -y'
RUN catkin build
