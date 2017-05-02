#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from tf.transformations import quaternion_from_euler
import math
from dynamic_reconfigure.server import Server
from drone_arena.cfg import ImuConfig


class IMU(object):
    """docstring for IMU."""
    def __init__(self):
        rospy.init_node('imu')
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'base_link'
        self.imu_msg.angular_velocity_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1
        self._std_dev = None
        self.angle_std_dev = rospy.get_param('angle_std_dev', 0.01)
        self.angle_bias = rospy.get_param('angle_bias', 0.0)
        self.srv = Server(ImuConfig, self.reconfigure)
        self.pub = rospy.Publisher('imu', Imu, queue_size=1)
        rospy.Subscriber('states/ardrone3/PilotingState/AttitudeChanged',
                         Ardrone3PilotingStateAttitudeChanged, self.has_received_attitude)
        rospy.spin()

    def has_received_attitude(self, msg):
        # We define base_link oriented towards the camera and utm oriented towards east
        # The driver has yaw = 0 pointing north.
        # print(msg.roll, msg.pitch, -msg.yaw + math.pi * 0.5)
        q = quaternion_from_euler(msg.roll, -msg.pitch, -msg.yaw + math.pi + self.angle_bias)
        self.imu_msg.orientation = Quaternion(*q)
        self.imu_msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.imu_msg)

    @property
    def angle_std_dev(self):
        return self._std_dev

    @angle_std_dev.setter
    def angle_std_dev(self, value):
        cov = [0] * 9
        var = value ** 2
        for i in range(3):
            cov[3 * i + i] = var
        self.imu_msg.orientation_covariance = cov
        self._std_dev = value

    def reconfigure(self, config, level):
        self.angle_std_dev = config['angle_std_dev']
        self.angle_bias = config['angle_bias']
        return config


if __name__ == '__main__':
    IMU()
