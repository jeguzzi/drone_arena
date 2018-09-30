#!/usr/bin/env python

from __future__ import division

import math

import numpy as np
import rospy
from std_msgs.msg import Bool, Empty, Header, String, UInt8

import actionlib
import diagnostic_msgs
import diagnostic_updater
import tf2_geometry_msgs
import tf2_ros
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from drone_arena.cfg import ArenaConfig
from drone_arena_msgs.msg import GoToPoseAction, GoToPoseFeedback, GoToPoseResult
from drone_arena.temporized import Temporized
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import (PointStamped, PoseStamped, Twist, TwistStamped,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
# from shapely.geometry import Polygon, Point
from tf.transformations import (euler_from_quaternion, quaternion_conjugate,
                                quaternion_from_euler, quaternion_multiply)

# from diagnostics import BebopDiagnostics


button = Temporized(1)


F = 2.83


def get_transform(tf_buffer, from_frame, to_frame):
    try:
        return tf_buffer.lookup_transform(
            from_frame, to_frame, rospy.Time(0), rospy.Duration(0.1)
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
        return None


def point_in_frame(tf_buffer, point_s, frame_id):
    t = get_transform(tf_buffer, frame_id, point_s.header.frame_id)
    if not t:
        return None
    return tf2_geometry_msgs.do_transform_point(point_s, t)


def pose_in_frame(tf_buffer, pose_s, frame_id):
    t = get_transform(tf_buffer, frame_id, pose_s.header.frame_id)
    if not t:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_s, t)


def twist_in_frame(tf_buffer, twist_s, frame_id):
    transform = get_transform(tf_buffer, frame_id, twist_s.header.frame_id)
    if not transform:
        return None
    h = Header(frame_id=frame_id, stamp=twist_s.header.stamp)
    vs_l = Vector3Stamped(header=h, vector=twist_s.twist.linear)
    vs_a = Vector3Stamped(header=h, vector=twist_s.twist.angular)
    msg = TwistStamped(header=h)
    msg.twist.linear = tf2_geometry_msgs.do_transform_vector3(vs_l, transform).vector
    msg.twist.angular = tf2_geometry_msgs.do_transform_vector3(vs_a, transform).vector
    return msg


def odometry_in_frame(tf_buffer, odom, frame_id, child_frame_id):
    # ignoring covariances
    transform_pose = get_transform(tf_buffer, frame_id, odom.header.frame_id)
    transform_twist = get_transform(tf_buffer, child_frame_id, odom.child_frame_id)
    if not (transform_pose and transform_twist):
        return None
    h = Header(frame_id=frame_id, stamp=odom.header.stamp)
    msg = Odometry(header=h, child_frame_id=child_frame_id)
    vs_l = Vector3Stamped(header=h, vector=odom.twist.twist.linear)
    vs_a = Vector3Stamped(header=h, vector=odom.twist.twist.angular)
    pose_s = PoseStamped(header=h, pose=odom.pose.pose)
    msg.twist.twist.linear = tf2_geometry_msgs.do_transform_vector3(vs_l, transform_twist).vector
    msg.twist.twist.angular = tf2_geometry_msgs.do_transform_vector3(vs_a, transform_twist).vector
    msg.pose.pose = tf2_geometry_msgs.do_transform_pose(pose_s, transform_pose).pose
    return msg


def is_stop(cmd_vel):
    a = np.array([cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z,
                  cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z])
    return np.allclose(a, np.array([0] * 6))


def rotate(v, q, inverse=False):
    '''rotate vector v from world to body frame (with quaternion q)'''
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


def cmd_from_angular_speed(omega):
    return min(max(omega / 1.75, -1), 1)


def clamp(xs, bss):
    return [max(min(x, bs[1]), bs[0]) for x, bs in zip(xs, bss)]


def target_yaw_to_observe(observer_point, target_point):
    d = np.array(target_point) - np.array(observer_point)
    return np.arctan2(d[1], d[0])


# full up/down = 70 degrees, i.e. from 60 = 25 + 35  to -60 = -25 - 35
def camera_control_cmd(observer_point, target_point):
    p1 = np.array(observer_point[:2])
    p2 = np.array(target_point[:2])
    d = np.linalg.norm(p1 - p2)
    delta_z = target_point[2] - observer_point[2]
    pitch_deg = math.atan2(delta_z, d) * 180.0 / math.pi
    cmd = Twist()
    cmd.angular.y = pitch_deg
    return cmd


class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        super(Controller, self).__init__()
        rospy.init_node('fence_control', anonymous=True)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.stop_pub = rospy.Publisher('stop', Empty, queue_size=1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.control_camera = rospy.get_param('control_camera', False)
        self.localization_pub = rospy.Publisher('location', String, queue_size=1, latch=True)
        self.camera_control_pub = rospy.Publisher(
            'camera_control', Twist, queue_size=1)
        self.land_home_pub = rospy.Publisher(
            'land_home', Empty, queue_size=1)
        self.pos_tol = rospy.get_param('position_tol', 0.1)
        self.angle_tol = rospy.get_param('angle_tol', 0.2)
        self.fence_margin = 0.5
        self.s_max = 0.5
        self.omega_max = 0.5
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

        self.frame_id = rospy.get_param('~frame_id', 'World')
        self.localization_timeout = rospy.get_param('~localization_timeout', 1)
        self.control_timeout = rospy.get_param('~control_timeout', 0.2)

        self.pos_bounds = rospy.get_param(
            "~pos_bounds",
            ((-1.8, 1.8), (-1.8, 1.8), (0.5, 2.0)))
        _home = rospy.get_param("~home", (0, 0, 1))
        self.home_p = _home
        self.home = PoseStamped()
        self.home.header.frame_id = self.frame_id
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
        self.target_velocity = [0, 0, 0]
        self.srv = Server(ArenaConfig, self.callback)
        self.localized = False
        self.localization_active = True
        self.last_localization = None
        self.publish_location()
        self._follow_vel_cmd = False

        self.track_head = False
        self.track_range = 1.5

        self.track_vertical_head = rospy.get_param('~track_vertical_head', False)
        self.head_altitude_difference = rospy.get_param('~head_altitude_difference', -0.2)
        self.head_altitude = rospy.get_param('~head_altitude', 1.5)

        # self.battery_msg = None
        self.tracked_teleop_d = 1.0
        self.tracked_teleop = False

        self.observe = False
        self.observe_point = None
        self.head_point = None

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("bebop controller")
        self.updater.add("Localization", self.localization_diagnostics)
        # self.updater.add("Battery", self.battery_diagnostics)

        # self.diagnostics = BebopDiagnostics()

        rospy.Subscriber('cmd_vel_input', Twist, self.has_received_input_cmd)
        rospy.Subscriber('takeoff_input', Empty, button(self.has_received_takeoff))
        rospy.Subscriber('land', Empty, button(self.has_received_land))
        rospy.Subscriber('odom', Odometry, self.has_received_odometry)
        rospy.Subscriber('target', PoseStamped, self.has_received_target)
        rospy.Subscriber('des_body_vel', Twist, self.has_received_desired_body_vel)
        rospy.Subscriber('des_vel', TwistStamped, self.has_received_desired_vel)

        rospy.Subscriber('enable_tracking', Bool,
                         button(self.has_received_enable_tracking))
        rospy.Subscriber('/head/mocap_odom', Odometry,
                         self.has_received_head)
        rospy.Subscriber('states/common/CommonState/BatteryStateChanged',
                         CommonCommonStateBatteryStateChanged,
                         self.has_received_battery)

        rospy.Subscriber('localization_active', Bool, self.has_received_localization_active)

        rospy.Subscriber(
            'switch_observe', Empty, button(self.switch_observe), queue_size=1)
        rospy.Subscriber(
            'observe', UInt8, button(self.set_observe_from_msg), queue_size=1)
        rospy.Subscriber(
            'stop_observe', Empty, button(self.stop_observe), queue_size=1)
        rospy.Subscriber(
            'observe_point', PointStamped, self.start_observe, queue_size=1)

        rospy.Timer(rospy.Duration(1), self.update_diagnostics)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)

        rospy.loginfo('Will start SimpleActionServer fence_control')
        self._as = actionlib.SimpleActionServer(
            'fence_control', GoToPoseAction, execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()
        rospy.loginfo('Started SimpleActionServer fence_control')
        rospy.spin()

    def has_received_localization_active(self, msg):
        self.localization_active = msg.data

    def is_near(self, position, yaw):
        d_p = np.linalg.norm(np.array(position) - np.array(self.position))
        d_y = yaw - self.yaw
        if d_y > math.pi:
            d_y = d_y - 2 * math.pi
        if d_y < - math.pi:
            d_y = d_y + 2 * math.pi
        return (d_p < self.pos_tol and abs(d_y) < self.angle_tol, d_p, d_y)

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
            self.observe = False
            self.follow_vel_cmd = False
            # Will preempt GoToPose action

    def has_received_head(self, msg):
        _p = msg.pose.pose.position
        self.head_point = [_p.x, _p.y, _p.z * 0.5]
        if self.track_head and self.localized:
            # rospy.loginfo("track_frame")
            _o = msg.pose.pose.orientation
            _v = msg.twist.twist.linear
            v = [_v.x, _v.y, 0]
            if self.track_vertical_head:
                p = [_p.x, _p.y, _p.z + self.head_altitude_difference]
            else:
                p = [_p.x, _p.y, self.head_altitude]
            q = [_o.x, _o.y, _o.z, _o.w]
            _, _, yaw = euler_from_quaternion(q)
            q = quaternion_from_euler(0, 0, yaw)
            self.track_frame(p, q, v)

    def track_frame(self, position, rotation, velocity):
        self.target_yaw = target_yaw_to_observe(self.position, position)
        f = [self.track_range, 0, 0, 0]
        f = np.array(rotate(f, rotation, inverse=True)[:3])
        p = np.array(position)
        self.target_position = self.clamp_in_fence_pos(p + f)
        self.target_velocity = velocity

    # def battery_diagnostics(self, stat):
    #     if not self.battery_msg:
    #         stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
    #                      "No battery status received")
    #     else:
    #         d = (rospy.Time.now() - self.battery_msg.header.stamp).to_sec()
    #         p = self.battery_msg.percent
    #         if d > 60:
    #             stat.summary(
    #                 diagnostic_msgs.msg.DiagnosticStatus.WARN, "Not updated")
    #         elif p < 10:
    #             stat.summary(
    #                 diagnostic_msgs.msg.DiagnosticStatus.WARN, "Almost empty")
    #         else:
    #             stat.summary(
    #                 diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
    #         stat.add("percent", p)
    #         stat.add("last updated", "%.0f seconds ago" % d)
    #     return stat

    def localization_diagnostics(self, stat):
        if self.localized:
            if self.inside_fence:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Outside fence")
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
        self.rotation_tau = config['rotation_tau']
        self.delay = config['delay']
        a = config['max_acceleration']
        self.max_acc_bounds = ((-a, a), (-a, a))
        self.s_max = config['max_speed']
        self.omega_max = config['max_angular_speed']
        self.enable_fence = config['enable_fence']
        self.tracked_teleop = config['tracked_teleop']
        self.tracked_teleop_d = config['tracked_teleop_d']
        self.teleop_mode = config['teleop_mode']
        self.track_range = config['track_distance']
        self.localization_timeout = config['localization_timeout']
        self.control_timeout = config['control_timeout']
        self.pos_tol = config['position_tol']
        self.angle_tol = config['angle_tol']

        self.track_vertical_head = config['track_vertical_head']
        self.head_altitude_difference = config['head_altitude_difference']
        self.head_altitude = config['head_altitude']

        # print config['teleop_mode']
        # print dir(ArenaConfig)

        return config

    def publish_location(self):
        if not self.localized:
            self.localization_pub.publish('')
        elif not self.inside_fence:
            self.localization_pub.publish('out')
        else:
            self.localization_pub.publish('in')

    def update_localization_state(self):
        if self.localized:
            if (rospy.Time.now() - self.last_localization).to_sec() > self.localization_timeout:
                rospy.logwarn("No more localized")
                self.localized = False
                self.publish_location()
                self.pub_cmd.publish(Twist())

    def update_hovering_cmd(self):
        if self.latest_cmd_time:
            delta = rospy.Time.now() - self.latest_cmd_time
            if delta.to_sec() < self.control_timeout:
                self.hovering_cmd = False
                return
        self.hovering_cmd = True
        self._follow_vel_cmd = False

    def has_received_vel_in_world_frame(self, des_vel, des_omega):
        # The same as if a cmd would have been received
        self.target_position = None
        self.track_head = False
        self.follow_vel_cmd = True
        self.latest_cmd_time = rospy.Time.now()
        if not self.localized:
            return
        self.update_desired_velocity(des_vel, des_omega)

    def has_received_desired_body_vel(self, msg):
        # convert in world frame
        vel = [msg.linear.x, msg.linear.y, msg.linear.z, 0]
        omega = [msg.angular.x, msg.angular.y, msg.angular.z, 0]
        des_vel = rotate(vel, self.q, inverse=True)[:3]
        des_omega = rotate(omega, self.q, inverse=True)[2]
        self.has_received_vel_in_world_frame(des_vel, des_omega)

    def has_received_desired_vel(self, msg):
        # convert in world frame
        twist_s = twist_in_frame(self.tf_buffer, msg, self.frame_id)
        v = twist_s.twist.linear
        des_vel = [v.x, v.y, v.z]
        des_omega = twist_s.twist.angular.z
        # des_vel = tf2_geometry_msgs.do_transform_vector3(msg.twist.linear, transform)
        # des_omega = tf2_geometry_msgs.do_transform_vector3(msg.twist.angular, transform)[2]
        self.has_received_vel_in_world_frame(des_vel, des_omega)

    def update_desired_velocity(self, v_des, w_des):
        vz_des = v_des[2]
        v_des = np.array(v_des[:2])
        s_des = np.linalg.norm(v_des)
        if s_des > self.s_max:
            v_des = v_des / s_des * self.s_max
            # TODO velocita' diverse per xy e z
        a_des = (v_des - np.array(self.velocity[:2])) / self.tau
        a = clamp(a_des, self.max_acc_bounds)
        if self.enable_fence:
            a = clamp(a, self.acc_bounds)
        # a = clamp(a, self.max_acc_bounds)
        # if self.enable_fence:
        #   a = clamp(a, self.acc_bounds)
        # a = clamp(a_des, self.acc_bounds)
        t = cmd_from_acc(a, self.q)
        rospy.logdebug("v_des %s, a_des %s, t %s" % (v_des, a_des, t))
        cmd_vel = Twist()
        cmd_vel.linear.x = t[0]
        cmd_vel.linear.y = t[1]
        cmd_vel.linear.z = vz_des
        cmd_vel.angular.z = cmd_from_angular_speed(w_des)
        rospy.logdebug("-> %s" % cmd_vel)
        self.pub_cmd.publish(cmd_vel)

    def update_pose_control(self, target_position, target_yaw,
                            target_velocity=[0, 0, 0]):
        if self.observe:
            _y = self.observe_target_yaw()
            if _y is not None:
                target_yaw = _y
        rospy.logdebug(
            "Go to target %s %s %s", target_position, target_yaw,
            target_velocity)
        rospy.logdebug(
            "From %s %s %s" % (self.position, self.yaw, self.velocity))
        v_des = ((np.array(target_position) - np.array(self.position) -
                  np.array(self.velocity) * self.delay) /
                 self.eta + np.array(target_velocity))
        self.update_desired_velocity(v_des, self.target_angular_yaw(target_yaw))

    def target_angular_yaw(self, target_yaw):
        d_yaw = target_yaw - self.yaw
        if d_yaw > math.pi:
            d_yaw = d_yaw - 2 * math.pi
        if d_yaw < -math.pi:
            d_yaw = d_yaw + 2 * math.pi
        v_yaw = d_yaw / self.rotation_tau
        if abs(v_yaw) > self.omega_max:
            v_yaw = v_yaw / abs(v_yaw) * self.omega_max
        return v_yaw

    def update(self, evt):
        # print "*****"
        self.update_localization_state()
        self.update_hovering_cmd()
        if self.control_camera:
            self.update_camera_control()
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
                if self.observe:
                    self.update_observe(msg)
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
            self.observe = False
            self.target_position = self.home_p
            self.target_velocity = [0, 0, 0]
            self.follow_vel_cmd = False
            rospy.logwarn("flying home to %s" % self.home_p)

    def has_received_battery(self, msg):
        # self.battery_msg = msg
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
        self.observe = False
        self.follow_vel_cmd = False
        # self.pub_land.publish(Empty())

    def has_received_takeoff(self, msg):
        if self.inside_fence:
            self.target_position = None
            self.track_head = False
            self.observe = False
            self.follow_vel_cmd = False
            self.pub_takeoff.publish(Empty())
        else:
            rospy.logwarn("outside fence, will not takeoff")

    @property
    def follow_vel_cmd(self):
        return self._follow_vel_cmd

    @follow_vel_cmd.setter
    def follow_vel_cmd(self, value):
        if value != self._follow_vel_cmd:
            self._follow_vel_cmd = value
            if not value:
                self.stop_pub.publish()

    def has_received_input_cmd(self, msg):
        self.follow_vel_cmd = False
        self.target_position = None
        self.track_head = False
        self.latest_cmd_time = rospy.Time.now()
        if not self.enable_fence:
            self.pub_cmd.publish(msg)
            return
        if not self.localized:
            self.pub_cmd.publish(Twist())
            return
        vz = msg.linear.z
        msg.linear.z = min(msg.linear.z, 2 * (self.max_height - self.z))
        if self.observe:
            self.update_observe(msg)
        # rospy.logwarn("too high")
        if self.inside_fence:
            # TODO apply when outside too
            if self.tracked_teleop and not is_stop(msg):
                t_pos, t_yaw, t_v = self.tracked_teleop_cmd_v(msg, vz)
                self.update_pose_control(t_pos, t_yaw, t_v)
                return
            self.clamp_in_fence_cmd(msg)
            self.pub_cmd.publish(msg)
        else:
            self.pub_cmd.publish(Twist())
            rospy.logwarn("outside fence, will hover")

    def clamp_in_fence_pos(self, pos):
        if self.enable_fence:
            return clamp(pos, self.pos_bounds)
        else:
            return pos

    def tracked_teleop_cmd_v(self, twist_msg, vz):
        vyaw = twist_msg.angular.z
        # world frame
        acc = self.acc_from_teleop_cmd(twist_msg)
        a = np.array(acc[:2])
        v = a / np.linalg.norm(a) * self.s_max
        v = [v[0], v[1], vz]
        target_position = self.position
        target_yaw = self.yaw + vyaw * self.tracked_teleop_d
        target_v = v
        return target_position, target_yaw, target_v

    # def tracked_teleop_cmd_pos(self, acc, vz, vyaw):
    #     # world frame
    #     d = self.tracked_teleop_d
    #     a = np.array(acc[:2])
    #     v = a / np.linalg.norm(a) / F * self.s_max
    #     v = [v[0], v[1], vz]
    #     target_position = np.array(self.position) + np.array(v) * d
    #     target_yaw = self.yaw + vyaw * d
    #     return target_position, target_yaw

    def acc_from_teleop_cmd(self, cmd_vel):
        if self.teleop_mode == ArenaConfig.Arena_Head:
            transform = get_transform(self.tf_buffer, self.frame_id, self.head_frame_id)
            if transform:
                q = transform[1]
            else:
                q = None
            if q is not None:
                return acc_from_cmd(cmd_vel, q)
            else:
                return [0, 0]
        elif self.teleop_mode == ArenaConfig.Arena_World:
            return acc_from_cmd(cmd_vel, [0, 0, 0, 1])
        elif self.teleop_mode == ArenaConfig.Arena_Body:
            return acc_from_cmd(cmd_vel, self.q)

    def clamp_in_fence_cmd(self, cmd_vel):
        rospy.logdebug("clamp %s" % cmd_vel)
        a = self.acc_from_teleop_cmd(cmd_vel)
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
        self.observe = False
        self.follow_vel_cmd = False
        # convert in world frame
        target_pose = pose_in_frame(self.tf_buffer, msg, self.frame_id)
        if not target_pose:
            rospy.logwarn("has_received_target: Cannot transform pose %s to %s",
                          msg, self.frame_id)
            return
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

        if not self.localization_active:
            return

        # Transform pose to World and twist to world
        odom = odometry_in_frame(self.tf_buffer, msg, self.frame_id, self.frame_id)
        if not odom:
            return
        msg = odom
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

        self.publish_location()

    def set_observe_from_msg(self, msg):
        # rospy.loginfo('BANANANA')
        if msg.data == 0:
            rospy.loginfo("Observe head")
            self.observe = "head"
        if msg.data == 1:
            rospy.loginfo("Observe point")
            self.observe = "point"

    def switch_observe(self, msg):
        rospy.loginfo("Switch observe from %s", self.observe)
        if not self.observe:
            self.observe = "head"
            return
        if self.observe == "point":
            self.observe = "head"
            return
        if self.observe == "head" and self.observe_point:
            self.observe = "point"
            return

    def stop_observe(self, msg):
        rospy.loginfo("Stop observe")
        self.observe = False
        self.camera_control_pub.publish(Twist())
        pass

    def start_observe(self, msg):
        rospy.loginfo("Has received observe target")
        point_s = point_in_frame(self.tf_buffer, msg, self.frame_id)
        if not point_s:
            return
        _p = point_s.point
        self.observe_point = [_p.x, _p.y, _p.z]
        self.observe = "point"
        pass

    # replace the msg.angular.z with the appropriate value to point
    # to the target

    def observe_target(self):
        if self.observe == "point":
            return self.observe_point
        elif self.observe == "head":
            return self.head_point
        else:
            return None

    def observe_target_yaw(self):
        observe_point = self.observe_target()
        if observe_point is not None:
            return target_yaw_to_observe(self.position, observe_point)
        return None

    def update_observe(self, msg):
        yaw = self.observe_target_yaw()
        if yaw is not None:
            wz = cmd_from_angular_speed(self.target_angular_yaw(yaw))
            msg.angular.z = wz

    def update_camera_control(self):
        if not self.localized:
            self.pub_cmd.publish(Twist())
            return
        observe_point = self.observe_target()
        if observe_point is not None:
            cmd = camera_control_cmd(self.position, observe_point)
            self.camera_control_pub.publish(cmd)
            #  rospy.loginfo(cmd.angular)


if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
