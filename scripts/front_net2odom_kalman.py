#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from drone_arena.kalman import KalmanFilter, rotate
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32MultiArray
from typing import Optional
import numpy as np


def yaw(o: Quaternion) -> float:
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[-1]


def quaternion_with(yaw: float) -> Quaternion:
    q = quaternion_from_euler(0, 0, yaw)
    # print(yaw, q)
    return Quaternion(*q)


class KalmanFilterROS(KalmanFilter):

    def __init__(self) -> None:
        rospy.init_node('prediction2odom')
        rospy.Subscriber('prediction', Float32MultiArray, self.has_received_prediction,
                         queue_size=1)
        rospy.Subscriber('odom', Odometry, self.has_received_odom, queue_size=1)
        self.pub = rospy.Publisher('filtered_odom', Odometry, queue_size=1)
        q_v = rospy.get_param('~q_v', 2.7)
        q_omega = rospy.get_param('~q_omega', 5.3)
        r_x = rospy.get_param('~r_x', 0.018)
        r_y = rospy.get_param('~r_y', 0.009)
        r_phi = rospy.get_param('~r_phi', 0.08)
        self.odom_msg = Odometry()
        frame_id = rospy.get_param('~frame_id', 'cf/odom')
        self.odom_msg.header.frame_id = frame_id
        self.odom_msg.child_frame_id = frame_id
        self.pose: Optional[np.ndarray] = None
        super(KalmanFilterROS, self).__init__(
            q_v=q_v, q_omega=q_omega, r_x=r_x, r_y=r_y, r_phi=r_phi)
        rospy.loginfo("Ready")
        rospy.spin()

    def has_received_odom(self, msg: Odometry) -> None:
        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                              yaw(msg.pose.pose.orientation)])
        self.z = msg.pose.pose.position.z

    def has_received_prediction(self, msg: Float32MultiArray) -> None:
        if self.pose is None:
            rospy.logerr("Missing pose")
            return
        t = rospy.Time.now()
        self.odom_msg.header.stamp = t
        p = np.array(list(msg.data[:2]) + [np.pi + msg.data[-1]])
        r = rotate(self.pose[-1])
        z = (r @ p + self.pose)[:, np.newaxis]
        self.update(t.to_sec(), self.pose[-1], z)
        x = self._x

        # Partially done: Fill covariance
        self.odom_msg.pose.covariance[0] = self._p[0, 0]
        self.odom_msg.pose.covariance[1] = self._p[0, 1]
        self.odom_msg.pose.covariance[6] = self._p[1, 0]
        self.odom_msg.pose.covariance[7] = self._p[1, 1]
        self.odom_msg.pose.covariance[6 * 2 + 2] = 0.01
        self.odom_msg.pose.covariance[-1] = self._p[2, 2]
        self.odom_msg.pose.pose.position.x = x[0]
        self.odom_msg.pose.pose.position.y = x[1]
        # TODO: Extend to z
        self.odom_msg.pose.pose.position.z = msg.data[2] + self.z
        self.odom_msg.pose.pose.orientation = quaternion_with(yaw=float(x[2]))
        self.odom_msg.twist.twist.linear.x = x[3]
        self.odom_msg.twist.twist.linear.y = x[4]
        self.odom_msg.twist.twist.angular.z = x[5]
        self.pub.publish(self.odom_msg)


if __name__ == '__main__':
    KalmanFilterROS()
