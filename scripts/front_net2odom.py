#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math


def republish_font_net_output_as_odom(pub, frame_id='base_link'):
    odom = Odometry()
    odom.header.stamp = rospy.Time()
    odom.header.frame_id = frame_id
    odom.child_frame_id = frame_id
    ps = odom.pose.pose.position

    def f(msg):
        ps.x, ps.y, ps.z = msg.data[:3]
        theta = msg.data[-1] + math.pi
        odom.pose.pose.orientation.z = math.sin(0.5 * theta)
        odom.pose.pose.orientation.w = math.cos(0.5 * theta)
        pub.publish(odom)

    return f


if __name__ == '__main__':
    rospy.init_node('front_net2odom')
    pub = rospy.Publisher('odom', Odometry, queue_size=1)
    rospy.Subscriber('output', Float32MultiArray, republish_font_net_output_as_odom(pub),
                     queue_size=1)
    rospy.spin()
