#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
# import math


def republish_pose_as_odom(theta=None):
    odom = Odometry()
    pub = rospy.Publisher('odom', Odometry, queue_size=1)
    def f(msg):
        odom.header = msg.header
        odom.pose.pose = msg.pose
        odom.child_frame_id = msg.header.frame_id
        # if theta is not None:
        #     odom.pose.pose.orientation.w = math.cos(theta / 2)
        #     odom.pose.pose.orientation.z = math.sin(theta / 2)
        #     odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = 0
        pub.publish(odom)
    return f


if __name__ == '__main__':
    rospy.init_node('pose2odom')

    theta = rospy.get_param('~theta', None)
    rospy.Subscriber('pose', PoseStamped, republish_pose_as_odom(theta), queue_size=1)
    rospy.spin()
