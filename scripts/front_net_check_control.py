#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Quaternion, TwistStamped
from nav_msgs.msg import Odometry
from drone_arena.fence_control import fence_control, angular_control
from drone_arena.control import rotate, target_yaw_to_observe
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


def yaw(o: Quaternion) -> float:
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[-1]


class CheckControl:

    def __init__(self) -> None:
        rospy.init_node('check_control')
        self.maximal_speed = rospy.get_param('frontnet/max_speed', 1.0)
        self.maximal_angular_speed = rospy.get_param('frontnet/max_ang_speed', 1.0)
        self.maximal_vertical_speed = rospy.get_param('frontnet/max_vert_speed', 0.5)
        self.eta = rospy.get_param('frontnet/eta', 1.0)
        self.distance = rospy.get_param('frontnet/distance', 1.3)
        self.altitude = rospy.get_param('frontnet/altitude', 1.4)
        self.pub = rospy.Publisher('control', TwistStamped, queue_size=1)
        self.pose = np.array([0.0, 0.0, 0.0, 0.0])
        rospy.Subscriber('odom', Odometry, self.has_received_odom, queue_size=1)
        rospy.Subscriber('head_odom', Odometry, self.has_received_head_odom, queue_size=1)
        rospy.spin()

    def has_received_odom(self, msg: Odometry) -> None:
        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                              msg.pose.pose.position.z,
                              yaw(msg.pose.pose.orientation)])
        self.velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z])

    def has_received_head_odom(self, msg: Odometry) -> None:
        pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                         msg.pose.pose.position.z, yaw(msg.pose.pose.orientation)])
        velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                             msg.twist.twist.linear.z])
        _o = msg.pose.pose.orientation
        q = [_o.x, _o.y, _o.z, _o.w]
        _, _, _yaw = euler_from_quaternion(q)
        q = quaternion_from_euler(0, 0, _yaw)
        f = [self.distance, 0, self.altitude]
        f = np.array(rotate(f, q, inverse=True))
        target_position = pose[:3] + f
        target_yaw = target_yaw_to_observe(self.pose[:3], pose[:3])
        # rospy.loginfo(f"{self.pose[:3]} -> {target_position}; {self.pose[-1]} {target_yaw}")
        _, des_velocity, _ = fence_control(
            self.pose[:3], self.velocity, target_position, velocity,
            None, maximal_speed=self.maximal_speed,
            maximal_vertical_speed=self.maximal_vertical_speed, eta=self.eta,
            compute_velocity=True)
        _, des_angular_speed = angular_control(
            yaw=self.pose[-1], target_yaw=target_yaw, rotation_tau=self.eta,
            maximal_angular_speed=self.maximal_angular_speed)
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'cf/odom'
        v = msg.twist.linear
        v.x, v.y, v.z = des_velocity
        # deg/s
        msg.twist.angular.z = 180.0 * des_angular_speed / np.pi
        self.pub.publish(msg)


if __name__ == '__main__':
    CheckControl()
