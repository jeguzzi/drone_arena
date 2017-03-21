FROM jeguzzi/drone_arena:bebop
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

COPY . /root/catkin_ws/src/drone_arena
RUN git clone https://github.com/jeguzzi/optitrack_odom.git /root/catkin_ws/src/optitrack_odom
RUN git clone https://github.com/jeguzzi/optitrack_msgs.git /root/catkin_ws/src/optitrack_msgs
RUN catkin build
