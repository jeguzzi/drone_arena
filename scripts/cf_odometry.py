#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from crazyflie_driver.msg import GenericLogData
import message_filters

import tf


class CFOdometry(object):

    # TODO covariance

    def update_odom(self, msg1, msg2):
        odom_msg = Odometry()
        odom_msg.header.stamp = msg1.header.stamp
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.frame_id
        values1 = msg1.values
        values2 = msg2.values
        odom_msg.pose.pose.position = Point(*values1[0:3])
        odom_msg.pose.pose.orientation = Quaternion(*values2)
        odom_msg.twist.twist.linear = Vector3(*values1[3:])
        self.pub.publish(odom_msg)
        self.tf_broadcaster.sendTransform(values1[:3], values2, rospy.Time.now(), self.frame_id,
                                          self.odom_frame_id)

    def __init__(self):
        ns = rospy.get_param('~ns', None)
        frame_id = rospy.get_param('~frame_id', 'base_link')
        if ns:
            self.odom_frame_id = '{ns}/odom'.format(ns=ns)
            self.frame_id = '{ns}/{frame_id}'.format(ns=ns, frame_id=frame_id)
        else:
            self.odom_frame_id = 'odom'
            self.frame_id = frame_id

        rawodom = message_filters.Subscriber('rawodom', GenericLogData)
        rawodom2 = message_filters.Subscriber('rawodom2', GenericLogData)
        ts = message_filters.ApproximateTimeSynchronizer([rawodom, rawodom2], queue_size=10,
                                                         slop=0.02)
        ts.registerCallback(self.update_odom)
        self.pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()


if __name__ == '__main__':
    try:
        rospy.init_node('cf_odometry')
        CFOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
