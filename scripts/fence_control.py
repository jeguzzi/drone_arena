#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
# from shapely.geometry import Polygon, Point
from tf.transformations import quaternion_conjugate, quaternion_multiply
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

from dynamic_reconfigure.server import Server
from drone_arena.cfg import ArenaConfig

import diagnostic_updater
import diagnostic_msgs

import actionlib
from drone_arena.msg import GoToPoseFeedback, GoToPoseResult, GoToPoseAction


F = 2.83


def is_stop(cmd_vel):
    a = np.array([cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z,
                  cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z])
    return np.allclose(a, np.array([0] * 6))


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
        self.land_home_pub = rospy.Publisher(
            'land_home', Empty, queue_size=1)
        self.fence_margin = 0.5
        self.latest_cmd_time = None
        self.hovering_cmd = True
        # self.fence = Polygon(
        #     [(-2.8, -2.8), (-2.8, 2.8), (2.8, 2.8), (2.8, -2.8)])
        # self.max_height = 2.0
        self.inside_fence = False
        self.z = 0
        # self.x_min, self.x_max = (-2, 2)
        # self.y_min, self.y_max = (-2, 2)
        # self.pos_bounds = ((-1.8, 1.8), (-1.8, 1.8), (0.5, 2.0))
        self.pos_bounds = rospy.get_param(
            "~pos_bounds",
            ((-1.8, 1.8), (-1.8, 1.8), (0.5, 2.0)))
        _home = rospy.get_param("~home", (0, 0, 1))
        self.home_p = _home
        self.home = PoseStamped()
        self.home.header.frame_id = 'World'
        self.home.pose.position.x = _home[0]
        self.home.pose.position.y = _home[1]
        self.home.pose.position.z = _home[2]
        self.home.pose.orientation.w = 1

        self.head_frame_id = rospy.get_param('head_frame_id', 'head')
        self.acc_bounds = ((0, 0), (0, 0))
        # self.max_acc_bounds = ((-2, 2), (-2, 2))
        # self.tau = 0.5
        # self.eta = 1
        # self.delay = 0.3
        self.tf_buffer = tf2_ros.Buffer()  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_position = None
        self.target_yaw = 0.0
        # self.s_max = 1.5
        self.target_velocity = [0, 0, 0]
        self.srv = Server(ArenaConfig, self.callback)
        self.localized = False
        self.last_localization = None

        self.track_head = False
        self.track_range = 1.5

        self.battery_msg = None
        self.tracked_teleop_d = 1.0
        self.tracked_teleop = False
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("bebop controller")
        self.updater.add("Localization", self.localization_diagnostics)
        self.updater.add("Battery", self.battery_diagnostics)

        rospy.Subscriber('cmd_vel_input', Twist, self.has_received_input_cmd)
        rospy.Subscriber('takeoff_input', Empty, self.has_received_takeoff)
        rospy.Subscriber('land', Empty, self.has_received_land)
        rospy.Subscriber('odom', Odometry, self.has_received_odometry)
        rospy.Subscriber('target', PoseStamped, self.has_received_target)
        rospy.Subscriber('enable_tracking', Bool,
                         self.has_received_enable_tracking)
        rospy.Subscriber('/head/mocap_odom', Odometry,
                         self.has_received_head)
        rospy.Subscriber('states/common/CommonState/BatteryStateChanged',
                         CommonCommonStateBatteryStateChanged,
                         self.has_received_battery)

        rospy.Timer(rospy.Duration(1), self.update_diagnostics)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)

        self._as = actionlib.SimpleActionServer(
            'fence_control', GoToPoseAction, execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()
        rospy.spin()

    def is_near(self, position, yaw):
        d_p = np.linalg.norm(np.array(position) - np.array(self.position))
        d_y = yaw - self.yaw
        if d_y > math.pi:
            d_y = d_y - 2 * math.pi
        if d_y < - math.pi:
            d_y = d_y + 2 * math.pi
        return (d_p < 0.1 and d_y < 0.2, d_p, d_y)

    def execute_cb(self, goal):
        rospy.loginfo('Executing action Go To Pose %s' % goal.target_pose)
        self.go_to_pose(goal.target_pose)

    def go_to_pose(self, target_pose):
        self.has_received_target(target_pose)
        p = self.target_position
        y = self.target_yaw
        r = rospy.Rate(5.0)
        feedback = GoToPoseFeedback()
        result = GoToPoseResult()
        while(self.target_position and self.localized and not self.track_head):
            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted')
                self._as.set_preempted()
                self.target_position = None
                self.pub_cmd.publish(Twist())
                return
            near, feedback.distance, _ = self.is_near(p, y)
            if near:
                rospy.loginfo('Succeeded')
                self._as.set_succeeded(result)
                self.target_position = None
                return
            # publish the feedback
            self._as.publish_feedback(feedback)
            r.sleep()
        if not (self.target_position and self.localized and
                not self.track_head):
            self._as.set_preempted()
        self.target_position = None

    def has_received_enable_tracking(self, msg):
        self.track_head = msg.data
        if self.track_head:
            self.target_position = None
            # Will preempt GoToPose action

    def has_received_head(self, msg):
        if self.track_head and self.localized:
            # rospy.loginfo("track_frame")
            _p = msg.pose.pose.position
            _o = msg.pose.pose.orientation
            _v = msg.twist.twist.linear
            v = [_v.x, _v.y, 0]
            p = [_p.x, _p.y, 1.5]
            q = [_o.x, _o.y, _o.z, _o.w]
            _, _, yaw = euler_from_quaternion(q)
            q = quaternion_from_euler(0, 0, yaw)
            self.track_frame(p, q, v)

    def track_frame(self, position, rotation, velocity):
        p = np.array(position)
        d = p - np.array(self.position)
        self.target_yaw = np.arctan2(d[1], d[0])
        f = [self.track_range, 0, 0, 0]
        f = np.array(rotate(f, rotation, inverse=True)[:3])
        self.target_position = self.clamp_in_fence_pos(p + f)
        self.target_velocity = velocity

    def battery_diagnostics(self, stat):
        if not self.battery_msg:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         "No battery status received")
        else:
            d = (rospy.Time.now() - self.battery_msg.header.stamp).to_sec()
            p = self.battery_msg.percent
            if d > 60:
                stat.summary(
                    diagnostic_msgs.msg.DiagnosticStatus.WARN, "Not updated")
            elif p < 10:
                stat.summary(
                    diagnostic_msgs.msg.DiagnosticStatus.WARN, "Almost empty")
            else:
                stat.summary(
                    diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
            stat.add("percent", p)
            stat.add("last updated", "%.0f seconds ago" % d)
        return stat

    def localization_diagnostics(self, stat):
        if self.localized:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
        else:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Not localized")

    def update_diagnostics(self, event):
        self.updater.update()

    @property
    def max_height(self):
        return self.pos_bounds[2][1]

    def callback(self, config, level):
        self.eta = config['eta']
        self.tau = config['tau']
        self.delay = config['delay']
        a = config['max_acceleration']
        self.max_acc_bounds = ((-a, a), (-a, a))
        self.s_max = config['max_speed']
        self.enable_fence = config['enable_fence']
        self.tracked_teleop = config['tracked_teleop']
        self.tracked_teleop_d = config['tracked_teleop_d']
        self.teleop_mode = config['teleop_mode']
        self.track_range = config['track_distance']

        # print config['teleop_mode']
        # print dir(ArenaConfig)

        return config

    def update_localization_state(self):
        if self.localized:
            if (rospy.Time.now() - self.last_localization).to_sec() > 1:
                rospy.logwarn("No more localized")
                self.localized = False
                self.pub_cmd.publish(Twist())

    def update_hovering_cmd(self):
        if self.latest_cmd_time:
            delta = rospy.Time.now() - self.latest_cmd_time
            if delta.to_sec() < 0.2:
                self.hovering_cmd = False
                return
        self.hovering_cmd = True

    def update_pose_control(self, target_position, target_yaw,
                            target_velocity=[0, 0, 0]):
        rospy.logdebug(
            "Go to target %s %s %s", target_position, target_yaw,
            target_velocity)
        rospy.logdebug(
            "From %s %s %s" % (self.position, self.yaw, self.velocity))
        v_des = ((np.array(target_position) - np.array(self.position) -
                  np.array(self.velocity) * self.delay) /
                 self.eta + np.array(target_velocity))
        s_des = np.linalg.norm(v_des)
        if s_des > self.s_max:
            v_des = v_des / s_des * self.s_max
            # TODO velocita' diverse per xy e z
        a_des = (v_des - np.array(self.velocity)) / self.tau
        # a = clamp(a, self.max_acc_bounds)
        # if self.enable_fence:
        #   a = clamp(a, self.acc_bounds)
        a = clamp(a_des[:2], self.acc_bounds)
        t = cmd_from_acc(a, self.q)
        rospy.logdebug("v_des %s, a_des %s, t %s" % (v_des, a_des, t))
        cmd_vel = Twist()
        cmd_vel.linear.x = t[0]
        cmd_vel.linear.y = t[1]
        cmd_vel.linear.z = v_des[2]
        d_yaw = target_yaw - self.yaw
        if d_yaw > math.pi:
            d_yaw = d_yaw - 2 * math.pi
        if d_yaw < -math.pi:
            d_yaw = d_yaw + 2 * math.pi
        v_yaw = d_yaw / self.tau
        cmd_vel.angular.z = v_yaw
        rospy.logdebug("-> %s" % cmd_vel)
        self.pub_cmd.publish(cmd_vel)

    def update(self, evt):
        # print "*****"
        self.update_localization_state()
        self.update_hovering_cmd()
        # rospy.loginfo("update %s %s", self.target_position, self.localized)
        #
        if self.localized:
            if self.target_position is not None:
                self.update_pose_control(self.target_position, self.target_yaw,
                                         self.target_velocity)
            elif self.hovering_cmd:
                msg = Twist()
                msg.linear.z = min(0, 2 * (self.max_height - self.z))
                self.clamp_in_fence_cmd(msg)
                self.pub_cmd.publish(msg)

    # def go_home_and_land(self, msg):
    #     if self.home:
    #         rospy.logwarn("flying home to %s" % self.home)
    #         self.go_to_pose(self.home)
    #         self.target_position = None
    #         self.pub_land.publish(Empty())

    def go_home_and_land(self):
        self.land_home_pub.publish(Empty())

    def go_home(self):
        if self.home:
            self.track_head = False
            self.target_position = self.home_p
            self.target_velocity = [0, 0, 0]
            rospy.logwarn("flying home to %s" % self.home_p)

    def has_received_battery(self, msg):
        self.battery_msg = msg
        # if(msg.percent < 2):
        #     # not really usefull
        #     self.target_position = None
        #     self.track_head = False
        #     self.pub_land.publish(Empty())
        if(msg.percent < 5):
            rospy.logwarn("battery nearly deplated")
            self.go_home_and_land()

    def has_received_land(self, msg):
        rospy.loginfo("Controller has received landing")
        self.target_position = None
        self.track_head = False
        # self.pub_land.publish(Empty())

    def has_received_takeoff(self, msg):
        if self.inside_fence:
            self.target_position = None
            self.track_head = False
            self.pub_takeoff.publish(Empty())
        else:
            rospy.logwarn("outside fence, will not takeoff")

    def has_received_input_cmd(self, msg):
        self.target_position = None
        self.track_head = False
        self.latest_cmd_time = msg.header.stamp
        if not self.enable_fence:
            self.pub_cmd.publish(msg)
            return
        if not self.localized:
            self.pub_cmd.publish(Twist())
            return
        vz = msg.linear.z
        vyaw = msg.angular.z
        msg.linear.z = min(msg.linear.z, 2 * (self.max_height - self.z))
        # rospy.logwarn("too high")
        if self.inside_fence:
            # TODO apply when outside too
            a = self.clamp_in_fence_cmd(msg)
            if self.tracked_teleop and not is_stop(msg):
                t_pos, t_yaw, t_v = self.tracked_teleop_cmd_v(a, vz, vyaw)
                self.update_pose_control(t_pos, t_yaw, t_v)
                return
            self.pub_cmd.publish(msg)
        else:
            self.pub_cmd.publish(Twist())
            rospy.logwarn("outside fence, will hover")

    def clamp_in_fence_pos(self, pos):
        if self.enable_fence:
            return clamp(pos, self.pos_bounds)
        else:
            return pos

    def tracked_teleop_cmd_v(self, acc, vz, vyaw):
        # world frame
        a = np.array(acc[:2])
        v = a / np.linalg.norm(a) / F * self.s_max
        v = [v[0], v[1], vz]
        target_position = self.position
        target_yaw = self.yaw + vyaw * self.tracked_teleop_d
        target_v = v
        return target_position, target_yaw, target_v

    def tracked_teleop_cmd_pos(self, acc, vz, vyaw):
        # world frame
        d = self.tracked_teleop_d
        a = np.array(acc[:2])
        v = a / np.linalg.norm(a) / F * self.s_max
        v = [v[0], v[1], vz]
        target_position = np.array(self.position) + np.array(v) * d
        target_yaw = self.yaw + vyaw * d
        return target_position, target_yaw

    def clamp_in_fence_cmd(self, cmd_vel):
        rospy.logdebug("clamp %s" % cmd_vel)
        if self.teleop_mode == ArenaConfig.Arena_Head:
            try:
                _, q = self.tf_buffer.lookup_transform(
                    "World", self.head_frame_id, rospy.Time(0),
                    rospy.Duration(0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                q = None
            if q is not None:
                a = acc_from_cmd(cmd_vel, q)
            else:
                a = [0, 0]
        elif self.teleop_mode == ArenaConfig.Arena_World:
            a = acc_from_cmd(cmd_vel, [0, 0, 0, 1])
        elif self.teleop_mode == ArenaConfig.Arena_Body:
            a = acc_from_cmd(cmd_vel, self.q)
        rospy.logdebug("acc %s", a)
        a = clamp(a, self.max_acc_bounds)
        if self.enable_fence:
            a = clamp(a, self.acc_bounds)
        rospy.logdebug("clamped acc %s", a)
        t = cmd_from_acc(a, self.q)
        rospy.logdebug("clamped twist %s", t)
        cmd_vel.linear.x = t[0]
        cmd_vel.linear.y = t[1]
        rospy.logdebug("to %s" % cmd_vel)
        return a

    # TODO: force target to be inside the fence !!!
    # (shapely, maybe with a margin)

    def has_received_target(self, msg):
        rospy.loginfo("Has received target")
        self.track_head = False
        # convert in world frame
        try:
            transform = self.tf_buffer.lookup_transform(
                "World", msg.header.frame_id, rospy.Time(0),
                rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.target_position = None
        target_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
        _p = target_pose.pose.position
        _o = target_pose.pose.orientation
        self.target_position = self.clamp_in_fence_pos([_p.x, _p.y, _p.z])
        _, _, self.target_yaw = euler_from_quaternion([_o.x, _o.y, _o.z, _o.w])
        self.target_velocity = [0, 0, 0]

    def inside_fence_margin(self, position):
        m = self.fence_margin
        return all([x > x_min - m and x < x_max + m
                    for x, (x_min, x_max)
                    in zip(position[:2], self.pos_bounds[:2])])

    def has_received_odometry(self, msg):
        self.last_localization = msg.header.stamp
        _p = msg.pose.pose.position
        self.z = _p.z
        _v = msg.twist.twist.linear
        o = msg.pose.pose.orientation
        # position in world_frame
        self.position = [_p.x, _p.y, _p.z]
        self.inside_fence = self.inside_fence_margin(self.position)
        self.q = [o.x, o.y, o.z, o.w]
        _, _, self.yaw = euler_from_quaternion(self.q)
        # velocity in world frame
        # self.velocity =
        # rotate([_v.x, _v.y, _v.z, 0], self.q, inverse=True)[:2]
        self.velocity = [_v.x, _v.y, _v.z]
        self.acc_bounds = acceleration_bounds(
            self.pos_bounds[:2], self.position[:2], self.velocity[:2],
            self.tau, self.eta, self.delay)
        # rospy.loginfo("position %s", self.position)
        # rospy.loginfo("velocity %s", self.velocity)

        # rospy.loginfo(
        #     ("inside fence %s. acc bounds %s" %
        #      (self.inside_fence, self.acc_bounds)))
        self.localized = True


if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
