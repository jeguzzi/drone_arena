FROM jeguzzi/drone_arena:photo
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

COPY hl_navigation_msgs src/hl_navigation_msgs
COPY . src/drone_arena
RUN catkin build
