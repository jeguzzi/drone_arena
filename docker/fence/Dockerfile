FROM bebop:latest
MAINTAINER Jerome Guzzi "jerome@idsia.ch"


# RUN apt-get update && apt-get install -y \
#    git \
#    build-essential python-rosdep python-catkin-tools \
#    sudo

WORKDIR /root
COPY . /root/catkin_ws/src/drone_arena
RUN git clone https://github.com/jeguzzi/optitrack_odom.git /root/catkin_ws/src/optitrack_odom
RUN git clone https://github.com/jeguzzi/optitrack_msgs.git /root/catkin_ws/src/optitrack_msgs
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd catkin_ws; catkin build'
