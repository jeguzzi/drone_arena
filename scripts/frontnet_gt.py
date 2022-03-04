#! /usr/bin/env python
# type: ignore
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import math
import tf2_ros
from tf.transformations import euler_from_quaternion


def ground_truth(tf_buffer, reference_frame, object_frame):
    try:
        tf = tf_buffer.lookup_transform(
            reference_frame, object_frame, rospy.Time(0), rospy.Duration(0.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None
    p = tf.transform.translation
    o = tf.transform.rotation
    q = [o.x, o.y, o.z, o.w]
    _, _, yaw = euler_from_quaternion(q)
    yaw -= math.pi
    if yaw < - math.pi:
        yaw += 2 * math.pi
    return [p.x, p.y, p.z, yaw]


def update(pub, tf_buffer, reference_frame, object_frame):
    msg = Float32MultiArray()
    msg.layout.dim = [MultiArrayDimension(size=4, stride=1)]

    def f(evt):
        value = ground_truth(tf_buffer, reference_frame, object_frame)
        if value:
            msg.data = value
            pub.publish(msg)

    return f


def main():
    rospy.init_node('frontnet_gt')
    tf_buffer = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(tf_buffer)
    reference_frame = rospy.get_param('~reference', 'cf/base_stabilized')
    object_frame = rospy.get_param('~object', 'head')
    period = rospy.get_param('~period', 0.05)
    pub = rospy.Publisher('output_ground_truth', Float32MultiArray, queue_size=1)
    rospy.Timer(rospy.Duration(period), update(pub, tf_buffer, reference_frame, object_frame))
    rospy.spin()


if __name__ == '__main__':
    main()
