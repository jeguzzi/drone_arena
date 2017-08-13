FROM jeguzzi/drone_arena:nav
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN git clone https://github.com/lrse/ros-keyboard.git src/ros-keyboard

RUN apt-get update && apt-get install -y \
   libsdl1.2-dev \
   && rm -rf /var/lib/apt/lists/*

COPY . src/drone_arena

RUN catkin build
