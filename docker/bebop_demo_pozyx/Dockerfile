FROM jeguzzi/drone_arena:photo
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y --fix-missing\
   ros-kinetic-robot-localization \
   ros-kinetic-mavros-msgs \
   && rm -rf /var/lib/apt/lists/*

COPY . src/drone_arena
RUN catkin build
