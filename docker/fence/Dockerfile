FROM bebop:latest
MAINTAINER Jerome Guzzi "jerome@idsia.ch"


RUN apt-get update && apt-get install -y \
   git \
   build-essential python-rosdep python-catkin-tools \
   sudo

WORKDIR /root

RUN mkdir -p catkin_ws/src
RUN /bin/bash -c 'cd catkin_ws; git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy'

RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; catkin build'
# RUN /bin/bash -c 'cd catkin_ws; source ./devel/setup.bash; catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo;'
#
#
RUN /bin/sed -i \
    '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/root\/catkin_ws\/devel\/setup.bash"' \
    /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

RUN apt-get update && apt-get install -y \
   ros-kinetic-robot-state-publisher
