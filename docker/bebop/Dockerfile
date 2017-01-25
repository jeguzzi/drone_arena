FROM ros:latest
MAINTAINER Jerome Guzzi "jerome@idsia.ch"


RUN apt-get update && apt-get install -y \
   git \
   build-essential python-rosdep python-catkin-tools


RUN mkdir -p /home/root/catkin_ws/src
RUN /bin/bash -c 'cd /home/root/catkin_ws;
git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy;
rosdep update
rosdep install --from-paths src -i
catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo
'

RUN /bin/bash -c '. /opt/ros/jade/setup.bash; catkin_init_workspace /home/root/catkin_ws/src; catkin_make -C /home/root/catkin_ws;'
RUN /bin/sed -i \
    '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/home\/root\/catkin_ws\/devel\/setup.bash"' \
    /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
