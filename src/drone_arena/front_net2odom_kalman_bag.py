import rospy
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from drone_arena.kalman import KalmanFilter, rotate
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from typing import List
import numpy as np


def yaw(o: Quaternion) -> float:
    return euler_from_quaternion([o.x, o.y, o.z, o.w])[-1]


class KalmanFilterBag(KalmanFilter):

    def __init__(self) -> None:
        self.xs: List[np.ndarray] = []
        self._odom_topic = '/cf/odom'
        self._prediction_topic = '/cf/output'
        super(KalmanFilterBag, self).__init__(
            q_v=2.7, q_omega=5.3, r_x=0.018, r_y=0.009, r_phi=0.08)

    def read(self, path: str) -> None:
        bag = rosbag.Bag(path)
        topics = [self._odom_topic, self._prediction_topic]
        for topic, msg, stamp in bag.read_messages(topics=topics):
            if topic == self._odom_topic:
                self.has_received_odom(msg)
            if topic == self._prediction_topic:
                self.has_received_prediction(msg, stamp=stamp)

    def has_received_odom(self, msg: Odometry) -> None:
        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                              yaw(msg.pose.pose.orientation)])
        self.z = msg.pose.pose.position.z

    def has_received_prediction(self, msg: Float32MultiArray, stamp: rospy.Time
                                ) -> None:
        if self.pose is None:
            return
        p = np.array(list(msg.data[:2]) + [np.pi + msg.data[-1]])
        r = rotate(self.pose[-1])
        z = (r @ p + self.pose)[:, np.newaxis]
        self.update(stamp.to_sec(), self.pose[-1], z)
        self.xs.append(self._x)
