#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged


def has_received_altitude(data_z):
    def f(msg):
        data_z['data'] = msg.altitude
    return f


def republish_odometry(pub, pose_cov, twist_cov, z):
    def f(msg):
        msg.header.frame_id = 'map'
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        msg.pose.pose.position.z = z['data']
        pub.publish(msg)
    return f


if __name__ == '__main__':
    rospy.init_node('odom_cov')
    z = rospy.get_param('z', 0.01)
    speed = rospy.get_param('speed', 0.01)
    data_z = {'data': 0.0}
    pose_cov = [0] * 36
    for i in range(3):
        pose_cov[6 * i + i] = z * z
    twist_cov = [0] * 36
    twist_cov[0] = twist_cov[6 + 1] = twist_cov[6 * 2 + 2] = speed * speed
    pub = rospy.Publisher('output_odom', Odometry, queue_size=1)
    rospy.Subscriber('input_odom', Odometry, republish_odometry(pub, pose_cov, twist_cov, data_z))
    rospy.Subscriber('states/ardrone3/PilotingState/AltitudeChanged',
                     Ardrone3PilotingStateAltitudeChanged, has_received_altitude(data_z))
    rospy.spin()
