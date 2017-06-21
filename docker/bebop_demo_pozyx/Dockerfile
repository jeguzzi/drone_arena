FROM jeguzzi/drone_arena:photo
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y --fix-missing\
   ros-kinetic-robot-localization \
   ros-kinetic-mavros-msgs \
   python-pip \
   libusb-1.0-0-dev \
   && rm -rf /var/lib/apt/lists/*


RUN pip install blinkstick utm

RUN git clone https://github.com/jeguzzi/lawa.git /root/catkin_ws/src/lawa

COPY . src/drone_arena
RUN catkin build
