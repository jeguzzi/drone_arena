#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from shapely.geometry import Polygon, Point
from tf.transformations import quaternion_conjugate, quaternion_multiply
from tf.transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

F = 2.83


# rotate from world to body frame (with quaternion q)
def rotate(v, q, inverse=False):
    cq = quaternion_conjugate(q)
    if inverse:
        z = quaternion_multiply(q, v)
        return quaternion_multiply(z, cq)
    else:
        z = quaternion_multiply(v, q)
        return quaternion_multiply(cq, z)


def _a(x_t, x, v, tau, eta, delay):
    return ((x_t - x - v * delay) / eta - v) / tau


# TODO: set min/max accelerations??? (cmd is already clamped to [-1,1])

def acceleration_bounds(position_bounds, position, velocity, tau, eta, delay):
    return [[_a(b, p, v, tau, eta, delay) for b in bs]
            for bs, p, v in zip(position_bounds, position, velocity)]


def acc_from_cmd(cmd_vel, q):
    a = [F * cmd_vel.linear.x, F * cmd_vel.linear.y, 0, 0]
    return rotate(a, q, inverse=True)[:2]


def cmd_from_acc(acc, q):
    t = [acc[0] / F, acc[1] / F, 0, 0]
    return rotate(t, q)[:2]


def clamp(xs, bss):
    return [max(min(x, bs[1]), bs[0]) for x, bs in zip(xs, bss)]


class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        super(Controller, self).__init__()
        rospy.init_node('fence_control', anonymous=True)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        rospy.Subscriber('cmd_vel_input', Twist, self.has_received_input_cmd)
        rospy.Subscriber('takeoff_input', Empty, self.has_received_takeoff)
        rospy.Subscriber('land_input', Empty, self.has_received_land)
        rospy.Subscriber('odom', Odometry, self.has_received_odometry)
        rospy.Subscriber('target', PoseStamped, self.has_received_target)
        rospy.Subscriber('states/common/CommonState/BatteryStateChanged',
                         CommonCommonStateBatteryStateChanged,
                         self.has_received_battery)
        self.fence = Polygon(
            [(-2.8, -2.8), (-2.8, 2.8), (2.8, 2.8), (2.8, -2.8)])
        self.max_height = 2.0
        self.inside_fence = False
        self.z = 0
        # self.x_min, self.x_max = (-2, 2)
        # self.y_min, self.y_max = (-2, 2)
        self.pos_bounds = ((-1.8, 1.8), (-1.8, 1.8), (0.5, 2.0))
        self.acc_bounds = ((0, 0), (0, 0))
        self.max_acc_bounds = ((-2, 2), (-2, 2))
        self.tau = 0.5
        self.eta = 1
        self.delay = 0.3

        self.tf_buffer = tf2_ros.Buffer()  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.target_position = None
        self.target_yaw = None
        self.s_max = 1.5
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)
        self.home = (0, 0, 1)
        rospy.spin()

    def update(self, evt):
        if self.target_position:
            rospy.loginfo("Go to target %s %s", self.target_position,
                          self.target_yaw)
            rospy.loginfo("From %s %s" % (self.position, self.yaw))
            v_des = ((np.array(self.target_position) - np.array(self.position))
                     / self.eta)
            s_des = np.linalg.norm(v_des)
            if s_des > self.s_max:
                v_des = v_des / s_des * self.s_max
                # TODO velocita' diverse per xy e z
            a_des = (v_des[:2] - np.array(self.velocity)) / self.tau
            a = clamp(a_des[:2], self.acc_bounds)
            t = cmd_from_acc(a, self.q)
            rospy.loginfo("v_des %s, a_des %s, t %s" % (v_des, a_des, t))
            cmd_vel = Twist()
            cmd_vel.linear.x = t[0]
            cmd_vel.linear.y = t[1]
            cmd_vel.linear.z = v_des[2]
            d_yaw = self.target_yaw - self.yaw
            if d_yaw > math.pi:
                d_yaw = d_yaw - 2*math.pi
            if d_yaw < -math.pi:
                d_yaw = d_yaw + 2*math.pi
            v_yaw = d_yaw / self.tau
            cmd_vel.angular.z = v_yaw
            rospy.loginfo("-> %s" % cmd_vel)
            self.pub_cmd.publish(cmd_vel)

    def go_home(self):
        if self.home:
            self.target_position = self.home
            rospy.warning("flying home to %s" % self.home)

    def has_received_battery(self, msg):
        if(msg.percent < 2):
            # not really usefull
            self.pub_land.publish(Empty())
        if(msg.percent < 5):
            rospy.warning("battery nearly deplated")
            self.go_home()

    def has_received_land(self, msg):
        self.target_position = None
        # self.pub_land.publish(Empty())

    def has_received_takeoff(self, msg):
        if self.inside_fence:
            self.pub_takeoff.publish(Empty())
        else:
            rospy.logwarn("outside fence, will not takeoff")

    def has_received_input_cmd(self, msg):
        self.target_position = None
        msg.linear.z = min(msg.linear.z, 2 * (self.max_height - self.z))
        # rospy.logwarn("too high")
        if self.inside_fence:
            # TODO apply when outside too
            self.clamp_in_fence_cmd(msg)
            self.pub_cmd.publish(msg)
        else:
            self.pub_cmd.publish(Twist())
            rospy.logwarn("outside fence, will hover")

    def clamp_in_fence_pos(self, pos):
        return clamp(pos, self.pos_bounds)

    def clamp_in_fence_cmd(self, cmd_vel):
        rospy.loginfo("clamp %s" % cmd_vel)
        a = acc_from_cmd(cmd_vel, self.q)
        print("acc %s" % a)
        a = clamp(a, self.max_acc_bounds)
        a = clamp(a, self.acc_bounds)
        print("clamped acc %s" % a)
        t = cmd_from_acc(a, self.q)
        print("clamped twist %s" % t)
        cmd_vel.linear.x = t[0]
        cmd_vel.linear.y = t[1]
        rospy.loginfo("to %s" % cmd_vel)

    # TODO: force target to be inside the fence !!!
    # (shapely, maybe with a margin)

    def has_received_target(self, msg):
        rospy.loginfo("Has received target")
        # convert in world frame
        transform = self.tf_buffer.lookup_transform(
            "World", msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
        target_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
        _p = target_pose.pose.position
        _o = target_pose.pose.orientation
        self.target_position = self.clamp_in_fence_pos([_p.x, _p.y, _p.z])
        _, _, self.target_yaw = euler_from_quaternion([_o.x, _o.y, _o.z, _o.w])

    def has_received_odometry(self, msg):
        _p = msg.pose.pose.position
        p = Point((_p.x, _p.y))
        self.z = _p.z
        # rospy.loginfo((self.z, self.max_height))
        self.inside_fence = self.fence.contains(p)
        # position = [p.x, p.y]
        _v = msg.twist.twist.linear
        o = msg.pose.pose.orientation
        # position in world_frame
        self.position = [_p.x, _p.y, _p.z]
        self.q = [o.x, o.y, o.z, o.w]
        _, _, self.yaw = euler_from_quaternion(self.q)
        # velocity in world frame
        # self.velocity =
        # rotate([_v.x, _v.y, _v.z, 0], self.q, inverse=True)[:2]
        self.velocity = [_v.x, _v.y]
        self.acc_bounds = acceleration_bounds(
            self.pos_bounds[:2], self.position[:2], self.velocity[:2],
            self.tau, self.eta, self.delay)
        # rospy.loginfo("position %s", self.position)
        # rospy.loginfo("velocity %s", self.velocity)

        # rospy.loginfo(
        #     ("inside fence %s. acc bounds %s" %
        #      (self.inside_fence, self.acc_bounds)))


if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
