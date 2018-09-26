FROM jeguzzi/ros:melodic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    python-pip \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-melodic-image-view \
    ros-melodic-urdf \
    ros-melodic-parrot-arsdk \
    ros-melodic-tf2 \
    ros-melodic-control-msgs \
    ros-melodic-tf2-ros \
    ros-melodic-joy \
    ros-melodic-image-transport \
    ros-melodic-roslint \
    ros-melodic-tf \
    ros-melodic-angles \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-xacro \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    ros-melodic-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

COPY teleop_tools src/teleop_tools
COPY bebop_autonomy src/bebop_autonomy
COPY optitrack_msgs src/optitrack_msgs
COPY optitrack_odom src/optitrack_odom

COPY drone_arena_msgs src/drone_arena_msgs
COPY crazyflie_ros src/crazyflie_ros
COPY drone_arena src/drone_arena

# RUN apt-get update && \
#     rosdep update && \
#     rosdep install --from-paths src -i -y

RUN catkin build
