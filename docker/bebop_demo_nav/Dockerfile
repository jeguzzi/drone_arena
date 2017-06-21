FROM jeguzzi/drone_arena:photo
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    ros-kinetic-mavros-msgs \
    python-pip \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install blinkstick

RUN git clone https://github.com/jeguzzi/lawa.git /root/catkin_ws/src/lawa

COPY hl_navigation_msgs src/hl_navigation_msgs
COPY . src/drone_arena

RUN catkin build
