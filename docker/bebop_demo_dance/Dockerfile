FROM jeguzzi/drone_arena:nav
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN git clone https://github.com/lrse/ros-keyboard.git src/ros-keyboard

RUN apt-get update && apt-get install -y \
   libsdl1.2-dev \
   && rm -rf /var/lib/apt/lists/*

COPY action src/drone_arena/action
COPY cfg src/drone_arena/cfg
COPY config src/drone_arena/config
COPY src src/drone_arena/src
COPY launch src/drone_arena/launch
COPY scripts src/drone_arena/scripts

RUN catkin build
