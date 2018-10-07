from __future__ import division

import math
from abc import ABCMeta, abstractmethod

import enum
import numpy as np

import actionlib
import diagnostic_msgs
import diagnostic_updater
import rospy
import rostopic
import std_srvs.srv
import tf2_geometry_msgs
import tf2_ros
from drone_arena.cfg import ArenaConfig
from drone_arena.temporized import Temporized
from drone_arena_msgs.msg import State as StateMsg
from drone_arena_msgs.msg import (GoToPoseAction, GoToPoseFeedback,  # noqa
                                  GoToPoseGoal, GoToPoseResult, TargetSource)
from dynamic_reconfigure.server import Server
# from dynamic_reconfigure.client import Client
from geometry_msgs.msg import (Point, PointStamped, PoseStamped,  # noqa
                               Quaternion, TransformStamped, Twist,
                               TwistStamped, Vector3, Vector3Stamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, Header, String
from tf.transformations import (euler_from_quaternion, quaternion_conjugate,
                                quaternion_from_euler, quaternion_multiply)
from typing import (Any, Callable, Dict, Generator, List, Optional,  # noqa
                    Tuple)

from .fence_control import angular_control, fence_control, inside_fence_margin


class BatteryState(enum.Enum):
    ok = 0
    critical = 1
    empty = 2


class TargetMode(enum.Enum):
    teleop = 1
    cmd = 2
    pos = 3
    vel = 4
    body_vel = 5
    odom = 6


class TargetAngleMode(enum.Enum):
    sync = 0
    teleop = 1
    cmd = 2
    point = 3
    vel = 4
    target_point = 5
    target_orientation = 6


class TeleopMode(enum.Enum):
    cmd = 0
    vel = 1


class TeleopFrame(enum.Enum):
    body = 0
    world = 1
    head = 2


class State(enum.Enum):
    landed = 0
    taking_off = 1
    hovering = 2
    flying = 3
    landing = 4
    # flying_home = 5


button = Temporized(1)


def if_flying(f):  # type:(Callable) -> Callable
    def g(self, *args, **kwargs):  # type: (Any, *Any, **Any)  -> None
        if self.state not in [State.flying, State.hovering]:
            return
        return f(self, *args, **kwargs)
    return g


# def if_battery_is_fine(f):
#     def g(self, *args, **kwargs):
#         if self.battery_state != BatteryState.ok:
#             return
#         return f(self, *args, **kwargs)
#     return g


def get_transform(tf_buffer, from_frame, to_frame
                  ):  # type: (tf2_ros.Buffer, str, str) -> Optional[TransformStamped]
    try:
        return tf_buffer.lookup_transform(
            from_frame, to_frame, rospy.Time(0), rospy.Duration(0.1)
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
        return None


def point_in_frame(tf_buffer, point_s, frame_id
                   ):  # type: (tf2_ros.Buffer, PointStamped, str) -> PointStamped
    t = get_transform(tf_buffer, frame_id, point_s.header.frame_id)
    if not t:
        return None
    return tf2_geometry_msgs.do_transform_point(point_s, t)


def pose_in_frame(tf_buffer, pose_s, frame_id
                  ):  # type: (tf2_ros.Buffer, PoseStamped, str) -> PoseStamped
    t = get_transform(tf_buffer, frame_id, pose_s.header.frame_id)
    if not t:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_s, t)


def twist_in_frame(tf_buffer, twist_s, frame_id
                   ):  # type: (tf2_ros.Buffer, TwistStamped, str) -> TwistStamped
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


def odometry_in_frame(tf_buffer, odom, frame_id, child_frame_id
                      ):  # type: (tf2_ros.Buffer, Odometry, str, str) -> Odometry
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


def is_stop(cmd_vel):  # type: (Twist) -> bool
    a = np.array([cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z,
                  cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z])
    return np.allclose(a, np.array([0] * 6))


def rotate(v, q, inverse=False):  # type: (np.ndarray, np.ndarray, bool) -> np.ndarray
    '''rotate vector v from world to body frame (with quaternion q)'''
    cq = quaternion_conjugate(q)
    v = [v[0], v[1], v[2], 0]
    if inverse:
        z = quaternion_multiply(q, v)
        return quaternion_multiply(z, cq)[:3]
    else:
        z = quaternion_multiply(v, q)
        return quaternion_multiply(cq, z)[:3]


def target_yaw_to_observe(observer_point, target_point):  # type: (np.ndarray, np.ndarray) -> bool
    d = np.array(target_point) - np.array(observer_point)
    return np.arctan2(d[1], d[0])


class Controller(object):
    __metaclass__ = ABCMeta

    """docstring for Controller"""
    def __init__(self):
        # type: () -> None

        super(Controller, self).__init__()
        rospy.init_node('fence_control', anonymous=True)
        self.srv = None

        self.tf_buffer = tf2_ros.Buffer()  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.teleop_mode = TeleopMode.cmd
        self.teleop_frame = TeleopFrame.body
        self._state = State.landed  # type: State

        self.state_pub = rospy.Publisher("flight_state", StateMsg, queue_size=1, latch=True)

        self.should_publish_cmd = rospy.get_param("~publish_cmd", True)
        self.should_publish_body_vel = rospy.get_param("~publish_body_vel", True)
        self.should_publish_target = rospy.get_param("~publish_target", True)
        self.des_cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.des_vel_pub = rospy.Publisher('des_vel', Twist, queue_size=1)
        self.des_pose_pub = rospy.Publisher('des_pose', PoseStamped, queue_size=1)
        self.repeat_same_des_pose = rospy.get_param("~repeat_des_pose", False)

        self.maximal_speed = rospy.get_param('max_speed', 0.5)
        self.maximal_vertical_speed = rospy.get_param('max_vertical_speed', 0.5)
        self.maximal_angular_speed = rospy.get_param('max_angular_speed', 0.5)
        self.maximal_acceleration = rospy.get_param('max_acceleration', 1)

        self.frame_id = rospy.get_param('~frame_id', 'World')
        self.head_frame_id = rospy.get_param('~head_frame_id', 'head')

        self.target_source_timeout = rospy.get_param('~target_source_timeout', 0.3)
        self.output_timeout = rospy.get_param('~output_timeout', 0.2)
        self.latest_source_time = None
        self.latest_output_time = None

        self.init_target()

        self.srv = Server(ArenaConfig, self.callback)
        self.config = None  # type: Optional[Dict[str, Any]]

        self.observe_point = None
        self.head_point = None

        self.init_battery()
        self.init_localization()
        self.init_fence()
        self.init_diagnostics()
        self.init_action_server()
        self.init_odom_following()
        self.init_teleop()

        self.giving_feedback = False

        rospy.Subscriber('safe_takeoff', Empty, button(self.has_received_takeoff), queue_size=1)
        rospy.Subscriber('safe_land', Empty, self.has_received_safe_land, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.has_received_odometry, queue_size=1)
        rospy.Subscriber('joy', Joy, self.has_received_joy, queue_size=1)
        rospy.Subscriber('hover', Empty, self.has_received_hover, queue_size=1)
        rospy.Subscriber('give_feedback', Empty, Temporized(5)(self.has_received_give_feedback),
                         queue_size=1)

        # rospy.Subscriber('target', PoseStamped, self.has_received_target)
        # rospy.Subscriber('target/body_vel', Twist, self.has_received_target_body_vel)
        # rospy.Subscriber('target/vel', TwistStamped, self.has_received_target_vel)
        # rospy.Subscriber('target/odom', Odometry, self.has_received_target_odom)
        # # TODO: /head/mocap_odom
        # rospy.Subscriber('target/cmd_vel', Twist, self.has_received_target_cmd)

        # rospy.Subscriber('target/enable', Bool,
        #                  button(self.has_received_enable_tracking), TargetMode.pos)
        # rospy.Subscriber('target/vel/enable', Bool,
        #                  button(self.has_received_enable_tracking), TargetMode.vel)
        # rospy.Subscriber('target/odom/enable', Bool,
        #                  button(self.has_received_enable_tracking), TargetMode.odom)

        # self.enable_target_pub = rospy.Publisher(
        #     'target/enable', Bool, queue_size=1, latch=True)
        # self.enable_vel_target_pub = rospy.Publisher(
        #     'target/vel/enable', Bool, queue_size=1, latch=True)
        # self.enable_odom_target_pub = rospy.Publisher(
        #     'target/odom/enable', Bool, queue_size=1, latch=True)

        rospy.Subscriber('target_source', TargetSource, self.has_received_target_source,
                         queue_size=1)

        period = rospy.get_param('~control_period', 0.05)
        self.timer = rospy.Timer(rospy.Duration(period), self.update)
        self.init_diagnostics()

# --------------- target

    def init_target(self):  # type: () -> None
        self.target_mode = TargetMode.teleop
        self.target_source_sub = None  # type: Optional[rospy.Subscriber]
        self.target_source_topic = None  # type: Optional[str]
        self.target_source_is_active = False
        self.output_pose_is_active = False
        self.reset_target()
        self.target_angle_mode = TargetAngleMode.sync
        self.target_yaw = None  # type: Optional[float]
        self.target_angular_speed = None  # type: Optional[float]
        self.target_position = None  # type: Optional[np.ndarray]
        self.target_velocity = None  # type: Optional[np.ndarray]

    def reset_target(self):  # type: () -> None
        self.target_position = None
        self.target_velocity = None
        self.target_acceleration = None
        self.target_yaw = None
        self.target_angular_speed = None
        self.target_source_is_active = False
        self.output_pose_is_active = False

    def has_received_target_source(self, msg):  # type: (TargetSource) -> None
        rospy.loginfo('has_received_target_source %s', msg)
        if msg.topic:
            topic = rospy.resolve_name(msg.topic)
        else:
            topic = ''
        if (msg.mode < 0 or msg.mode > 6) and topic:
            topic_class = rostopic.get_topic_type(topic)
            mode = self.target_mode_from_msg_type(topic_class) or TargetMode.teleop
            if topic_class is None:
                rospy.warn("Cannot recover the target mode from topic %s with msg class %s,"
                           " please specify a mode", topic, topic_class)
                return
        else:
            mode = TargetMode(msg.mode)
        self.set_target_source(mode, topic)

    def set_target_source(self, mode, topic):  # type: (TargetMode, str) -> None
        if mode == self.target_mode and topic == self.target_source_topic:
            return
        callback, msg_type = self.target_callback(mode)
        rospy.loginfo("mode %s, topic %s, cb %s, msg_type %s", mode, topic, callback, msg_type)
        if msg_type is None or mode is None or callback is None:
            mode = TargetMode.teleop
        if self.target_mode != mode:
            self.target_mode = mode
            self.reset_target()

        if self.srv and self.config:
            if(self.config['target_mode'] != mode.value or
               self.config['target_source_topic'] != topic):
                self.srv.update_configuration(
                    {'target_mode': mode.value, 'target_source_topic': topic})

        if topic == self.target_source_topic:
            return
        self.target_source_topic = topic
        if self.target_source_sub:
            self.target_source_sub.unregister()
        if topic and mode != TargetMode.teleop:
            self.target_source_sub = rospy.Subscriber(topic, msg_type, callback, queue_size=1)
        else:
            self.target_source_sub = None

    @staticmethod
    def target_mode_from_msg_type(msg):  # type: (Any) -> Optional[rospy.TargetMode]
        return {
            PoseStamped: TargetMode.pos,
            Twist: TargetMode.body_vel,
            TwistStamped: TargetMode.vel,
            Odometry: TargetMode.odom}.get(msg, None)

    def target_callback(self, mode
                        ):  # type: (TargetMode) -> Tuple[Optional[function], Any]
        return {
            TargetMode.pos: (self.has_received_target, PoseStamped),
            TargetMode.cmd: (self.has_received_target_cmd, Twist),
            TargetMode.vel: (self.has_received_target_vel, TwistStamped),
            TargetMode.body_vel: (self.has_received_target_body_vel, Twist),
            TargetMode.odom: (self.has_received_target_odom, Odometry)}.get(
                mode, (None, None))

    @property
    def effective_angular_target_mode(self):  # type: () -> TargetAngleMode
        if self.target_angle_mode == TargetAngleMode.sync:
            return {TargetMode.teleop: TargetAngleMode.teleop,
                    TargetMode.cmd: TargetAngleMode.cmd,
                    TargetMode.pos: TargetAngleMode.target_orientation,
                    TargetMode.vel: TargetAngleMode.vel,
                    TargetMode.body_vel: TargetAngleMode.vel,
                    TargetMode.odom: TargetAngleMode.target_point}[self.target_mode]
        return self.target_angle_mode

# --------------- teleop

    def init_teleop(self):  # type: () -> None
        self.deadman_button = rospy.get_param("~deadman", 7)
        self.joy_axes = rospy.get_param("~joy_axes", [3, 2, 1, 0])
        self.joy_set_teleop_mode = rospy.get_param("~joy_set_teleop_mode", True)

# --------------- battery

    def init_battery(self):  # type: () -> None
        self._battery_state = None  # type: Optional[BatteryState]
        self.battery_percent = None

    @property
    def battery_state(self):  # type: () -> Optional[BatteryState]
        return self._battery_state

    @battery_state.setter
    def battery_state(self, value):  # type: (Optional[BatteryState]) -> None
        if value != self._battery_state:
            self._battery_state = value
            rospy.loginfo("Battery state %s", value)
            if value == BatteryState.critical and self.state in [State.flying, State.hovering]:
                self.go_home_and_land()
            if value == BatteryState.empty and self.state not in [State.landing, State.landed]:
                self.land()

# --------------- dynamic reconfig

    def callback(self, config, level):  # type: (Dict[str, Any], int) -> Dict[str, Any]

        self.config = config
        self.eta = config['eta']
        self.tau = config['tau']
        self.rotation_tau = config['rotation_tau']
        self.delay = config['delay']
        self.pos_tol = config['position_tol']
        self.angle_tol = config['angle_tol']
        self.maximal_acceleration = config['max_acceleration']
        self.maximal_speed = config['max_speed']
        self.maximal_vertical_speed = config['max_vertical_speed']
        self.maximal_angular_speed = config['max_angular_speed']
        self.enforce_fence = config['enable_fence']
        self.teleop_mode = TeleopMode(config['teleop_mode'])
        self.teleop_frame = TeleopFrame(config['teleop_frame'])

        self.target_odom_r = config['track_distance']
        self.target_odom_z_is_relative = config['track_altitude_as_relative']
        self.target_odom_z = config['track_altitude']
        self.target_odom_rel_z = config['track_relative_altitude']
        self.target_odom_yaw = config['track_yaw']

        self.localization_timeout = config['localization_timeout']
        self.target_source_timeout = config['target_source_timeout']
        self.output_timeout = config['output_timeout']

        mode = TargetMode(config['target_mode'])
        topic = config['target_source_topic']
        self.set_target_source(mode, topic)
        self.target_angle_mode = TargetAngleMode(config['target_angle_mode'])

        self.should_publish_cmd = config["publish_cmd"]
        self.should_publish_body_vel = config["publish_body_vel"]
        self.should_publish_target = config["publish_target"]

        return config

# --------------- odom (head) following

    def init_odom_following(self):  # type: () -> None
        self.target_odom_r = rospy.get_param('~track_distance', False)
        self.target_odom_z_is_relative = rospy.get_param('~track_altitude_as_relative', False)
        self.target_odom_rel_z = rospy.get_param('~track_relative_altitude', -0.2)
        self.target_odom_z = rospy.get_param('~track_altitude', 1.5)
        self.target_odom_yaw = rospy.get_param('~track_yaw', 0)

# --------------- Diagnostics

    def init_diagnostics(self):  # type: () -> None
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("fence controller")
        self.updater.add("Localization", self.localization_diagnostics)
        self.updater.add("Battery", self.battery_diagnostics)
        self.updater.add("State", self.state_diagnostics)
        self.updater.add("Source", self.source_diagnostics)
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)

    def update_diagnostics(self, event):  # type: (rospy.TimerEvent) -> None
        self.updater.update()

    def localization_diagnostics(self, stat
                                 ):  # type: (diagnostic_updater.DiagnosticStatusWrapper) -> None
        if self.localized:
            if self.inside_fence:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Outside fence")
            if self.position:
                stat.add('x', self.position[0])
                stat.add('y', self.position[1])
                stat.add('z', self.position[2])
        else:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Not localized")

    def battery_diagnostics(self, stat):
        # type: (diagnostic_updater.DiagnosticStatusWrapper) -> None
        if self.battery_state is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "No info")
            return

        if self.battery_state == BatteryState.ok:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
        elif self.battery_state == BatteryState.critical:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Critical battery level")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Empty battery")
        if self.battery_percent is not None:
            stat.add('percentage', int(self.battery_percent))

    def source_diagnostics(self, stat
                           ):  # type: (diagnostic_updater.DiagnosticStatusWrapper) -> None
        if self.target_source_is_active:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "not active")
        stat.add('Target Mode', self.target_mode.name)
        stat.add('Target source topic', self.target_source_topic)
        tmode = self.target_angle_mode
        etmode = self.effective_angular_target_mode
        # rospy.loginfo("%s %s", tmode, etmode)
        if tmode != etmode:
            tmode_text = '{tmode.name} ({etmode.name})'.format(**locals())
        else:
            tmode_text = tmode.name
        stat.add('Angular Target Mode', tmode_text)

    def state_diagnostics(self, stat):
        # type: (diagnostic_updater.DiagnosticStatusWrapper) -> None
        if self.can_fly():
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, self.state.name)
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                         '{0} (not safe to fly)'.format(self.state.name))
        stat.add('battery is safe', self.battery_state == BatteryState.ok)
        stat.add('orientation is safe', self.orientation_is_safe)

# --------------- Localization

    def init_localization(self):  # type: () -> None
        self.pitch = self.roll = self.yaw = 0
        self.orientation_is_safe = False
        self.max_safe_pitch = self.max_safe_roll = rospy.get_param(
            '~max_safe_angle', math.pi * 0.5)
        self.localization_pub = rospy.Publisher('location', String, queue_size=1, latch=True)
        self._localized = None  # type: Optional[bool]
        self.localized = False
        self.localization_active = True
        self.last_localization = None
        self.localization_timeout = rospy.get_param('~localization_timeout', 1)
        rospy.Subscriber('localization_active', Bool, self.has_received_localization_active)

    @property
    def localized(self):  # type: () -> Optional[bool]
        return self._localized

    @localized.setter
    def localized(self, value):  # type: (bool) -> None
        if self._localized != value:
            self._localized = value
            self.publish_location()
            self.handle_non_localized()

    def publish_location(self):  # type: () -> None
        if not self.localized:
            self.localization_pub.publish('')
        elif not self.inside_fence:
            self.localization_pub.publish('out')
        else:
            self.localization_pub.publish('in')

    def has_received_localization_active(self, msg):  # type: (Bool) -> None
        self.localization_active = msg.data

    def update_localization_state(self):  # type: () -> None
        if self.localized:
            if (rospy.Time.now() - self.last_localization).to_sec() > self.localization_timeout:
                rospy.logwarn("No more localized")
                self.localized = False
                self.inside_fence = False

# --------------- fence

    @property
    def max_height(self):  # type: () -> float
        return self.pos_bounds[2][1]

    def init_fence(self):  # type: () -> None
        pos_bounds = rospy.get_param("~pos_bounds", ((-1.8, 1.8), (-1.8, 1.8), (0.5, 2.0)))
        self.pos_bounds = [(float(x), float(y)) for x, y in pos_bounds]
        _home = rospy.get_param("~home", None)
        if _home is not None:
            self.home = PoseStamped()
            self.home.header.frame_id = self.frame_id
            self.home.pose.position.x = _home[0]
            self.home.pose.position.y = _home[1]
            self.home.pose.position.z = _home[2]
            self.home.pose.orientation.w = 1
        else:
            self.home = None
        self.fence_margin = 0.5
        self.inside_fence = False
        self.enforce_fence = rospy.get_param('~enable_fence', True)

# --------------- Go to pose action

    def is_near(self, position, yaw):  # type: (np.ndarray, float) -> Tuple[bool, float, float]
        d_p = np.linalg.norm(np.array(position) - np.array(self.position))
        d_y = yaw - self.yaw  # type: ignore
        if d_y > math.pi:
            d_y = d_y - 2 * math.pi
        if d_y < - math.pi:
            d_y = d_y + 2 * math.pi
        # rospy.loginfo("%s (%s), %s (%s)", d_p, self.pos_tol, abs(d_y), self.angle_tol)
        return (d_p < self.pos_tol and abs(d_y) < self.angle_tol, d_p, d_y)

    def execute_cb(self, goal):  # type: (GoToPoseGoal) -> None
        rospy.loginfo('Go to pose %s' % goal.target_pose.pose)
        self.go_to_pose_action(goal.target_pose)

    def go_to_pose_no_feedback(self, target_pose):  # type: (PoseStamped) -> bool
        for near, _ in self.go_to_pose(target_pose):
            if near:
                return True
        return False

    def go_to_pose(self, target_pose):
        # type: (PoseStamped) -> Generator[Tuple[bool, float], None, None]

        if self.position is None or self.yaw is None:
            return

        self.set_target_source(TargetMode.pos, '')
        self.has_received_target(target_pose)
        p = self.target_position
        y = self.target_yaw

        if p is None or y is None:
            rospy.logerror("invalid target")
            return

        r = rospy.Rate(20)
        near = False
        while not near and self.target_position is not None and self.localized:
            near, distance, _ = self.is_near(p, y)
            yield (near, distance)
            if near:
                break
            r.sleep()
        if self.target_position is None:
            self.hover()
        rospy.loginfo("RESET target")
        self.reset_target()

    def go_to_pose_action(self, target_pose):   # type: (PoseStamped) -> None
        for near, distance in self.go_to_pose(target_pose):
            if self._as.is_preempt_requested():
                self.target_position = None
            else:
                self._as.publish_feedback(GoToPoseFeedback(distance=distance))
        if near:
            self._as.set_succeeded(GoToPoseResult())
        else:
            self._as.set_preempted()

    def init_action_server(self):
        # type: () -> None
        self.pos_tol = rospy.get_param('position_tol', 0.1)
        self.angle_tol = rospy.get_param('angle_tol', 0.2)
        rospy.loginfo('Will start SimpleActionServer fence_control')
        self._as = actionlib.SimpleActionServer(
            'fence_control', GoToPoseAction, execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()
        rospy.loginfo('Started SimpleActionServer fence_control')
        rospy.Service('safe_land', std_srvs.srv.Trigger, self.land_service)
        rospy.Service('safe_takeoff', std_srvs.srv.Trigger, self.takeoff_service)
        rospy.Service('give_feedback', std_srvs.srv.Trigger, self.give_feedback_service)
        rospy.loginfo('Started land/takeoff services')

    def land_service(self, req):
        # type: (std_srvs.srv.TriggerRequest) -> std_srvs.srv.TriggerResponse
        res = std_srvs.srv.TriggerResponse()
        res.success = self.blocking_land()
        return res

    def takeoff_service(self, req):
        # type: (std_srvs.srv.TriggerRequest) -> std_srvs.srv.TriggerResponse
        res = std_srvs.srv.TriggerResponse()
        res.success = self.blocking_takeoff()
        return res

    def give_feedback_service(self, req):
        # type: (std_srvs.srv.TriggerRequest) -> std_srvs.srv.TriggerResponse
        res = std_srvs.srv.TriggerResponse()
        res.success = self.blocking_give_feedback()
        return res

# ----------------- Controller

    @property
    def state(self):
        # type: () -> State
        return self._state

    @state.setter
    def state(self, value):
        # type: (State) -> None
        if self._state != value:
            # rospy.loginfo("flying state %s -> %s", self._state, value)
            self._state = value
            self.state_pub.publish(value.value)

    def handle_non_localized(self):  # type: () -> None
        pass

    def output_too_old(self):  # type: () -> bool
        if self.output_pose_is_active:
            return False
        if not self.latest_output_time:
            return False
        delta = rospy.Time.now() - self.latest_output_time
        if delta.to_sec() < self.output_timeout:
            return False
        return True

    def can_fly(self):  # type: () -> bool
        return self.orientation_is_safe

    def update_safety(self):  # type: () -> None
        if self.pitch is None or self.roll is None:
            return
        # rospy.loginfo('update_safety %s %s', self.pitch, self.roll)
        if abs(self.pitch) > self.max_safe_pitch or abs(self.roll) > self.max_safe_roll:
            self.orientation_is_safe = False
        else:
            self.orientation_is_safe = True

    def update(self, evt):  # type: (rospy.TimerEvent) -> None
        self.update_localization_state()
        self.update_source_timeout()
        self.update_safety()

        if not self.can_fly() and self.state != State.landed:
            self.stop()
            return

        if self.state == State.flying:
            if self.enforce_fence and self.localized and not self.inside_fence:
                rospy.logwarn("Outside fence => switch to hovering")
                self.hover()
                return
            if not self.target_source_is_active:
                rospy.logwarn("No recent control => switch to hovering")
                self.hover()
            if self.output_too_old():
                rospy.logwarn("No recent output => switch to hovering")
                self.hover()
                self.latest_output_time = None

        # rospy.loginfo("State %s, localized %s, target_mode %s, target_source_is_active %s",
        #               self.state, self.localized, self.target_mode, self.target_source_is_active)

        if(self.localized and self.target_mode != TargetMode.teleop and
           self.target_source_is_active and not self.giving_feedback):
            self.update_control()

    def update_source_timeout(self):  # type: () -> None
        if not self.target_source_is_active:
            return
        if self.target_mode == TargetMode.pos:
            return
        if self.latest_source_time:
            delta = rospy.Time.now() - self.latest_source_time
            if delta.to_sec() < self.target_source_timeout:
                return
        self.target_source_is_active = False

# ----------------- Flying
    @if_flying
    def update_control(self):  # type: () -> None

        # rospy.loginfo("State %s, target mode %s", self.state, self.target_mode)

        # if not self.inside_fence:
        #     self.hover()  # TODO: better safety
        #     return

        # rospy.loginfo("Target %s %s %s", self.target_position, self.target_velocity,
        #               self.target_acceleration)

        if self.enforce_fence:
            fence = self.pos_bounds  # type: Optional[List[Tuple[float, float]]]
        else:
            fence = None

        des_target, des_velocity, des_acceleration = fence_control(
            self.position, self.velocity, self.target_position, self.target_velocity,
            self.target_acceleration, delay=self.delay, fence=fence,
            maximal_acceleration=self.maximal_acceleration, maximal_speed=self.maximal_speed,
            maximal_vertical_speed=self.maximal_vertical_speed, eta=self.eta, tau=self.tau,
            compute_velocity=self.should_publish_body_vel,
            compute_acceleration=self.should_publish_cmd)

        # if self.target_angle_mode == TargetAngleMode.plane and des_velocity:
        #     if np.linalg.norm(des_velocity[:2]) > 0.1:
        #         self.target_yaw = np.arctan2(des_velocity[1], des_velocity[0])
        #         self.target_angular_speed = None
        #     else:
        #         self.target_yaw = None
        #         self.target_angular_speed = 0
        # rospy.loginfo("%s %s %s %s", self.yaw, self.target_yaw,
        # self.target_angular_speed, self.target_angle_mode)

        # rospy.loginfo("Target Angle %s %s", self.target_yaw, self.target_angular_speed)

        des_target_yaw, des_angular_speed = angular_control(
            self.yaw, self.target_yaw, self.target_angular_speed, rotation_tau=self.rotation_tau,
            maximal_angular_speed=self.maximal_angular_speed)

        # rospy.loginfo("des_target %s, des_velocity %s, des_acceleration %s",
        #               des_target, des_velocity, des_acceleration)

        output = False
        self.output_pose_is_active = True

        if self.should_publish_target and des_target is not None:
            # Maybe just a Point (lets see what cf needs)
            self.publish_target(des_target, des_target_yaw)
            self.output_pose_is_active = True
            output = True

        # rospy.loginfo("%s?", self.should_publish_body_vel)

        if not output and self.should_publish_body_vel and des_velocity is not None:
            vec = rotate(des_velocity, self.q, inverse=False)
            # rospy.loginfo("!")
            self.publish_target_body_vel(vec, des_angular_speed)
            self.output_pose_is_active = False
            output = True

        if not output and self.should_publish_cmd and des_acceleration is not None:
            vec = [des_acceleration[0], des_acceleration[1], 0]
            vec = rotate(vec, self.q, inverse=False)
            self.publish_target_cmd(vec[:2], des_velocity[2], des_angular_speed)
            self.output_pose_is_active = False
            output = True

        if output:
            self.latest_output_time = rospy.Time.now()

    def publish_target(self, des_target, des_target_yaw):  # type: (np.ndarray, float) -> None
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position = Point(*des_target)
        msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, des_target_yaw or 0))
        self.des_pose_pub.publish(msg)

    def publish_target_body_vel(self,
                                velocity, angular_speed):   # type: (np.ndarray, float) -> None
        msg = Twist()
        msg.linear = Vector3(*velocity)
        msg.angular = Vector3(0, 0, angular_speed)
        self.des_vel_pub.publish(msg)

    def publish_target_cmd(self, acc, vert_vel,
                           angular_speed):  # type: (np.ndarray, float, float) -> None
        msg = self.cmd_from(acc, vert_vel, angular_speed)
        self.des_cmd_pub.publish(msg)

    #
    # @property
    # def target_mode(self):
    #     return self._target_mode
    #
    # @target_mode.setter
    # def target_mode(self, value):
    #     if value != self._target_mode:
    #         rospy.loginfo('Change target mode from %s to %s', self._target_mode, value)
    #         prev_value = self._target_mode
    #         self._target_mode = value
    #         if prev_value == TargetMode.pos:
    #             self.enable_target_pub.publish(False)
    #         if prev_value == TargetMode.vel:
    #             self.enable_vel_target_pub.publish(False)
    #         if prev_value == TargetMode.odom:
    #             self.enable_odom_target_pub.publish(False)
    #
    #         if self.srv and self.config and self.config['target_mode'] != value.value:
    #             # rospy.loginfo("Propagate")
    #             self.srv.update_configuration({'target_mode': value.value})
    #         else:
    #             pass
    #             # rospy.loginfo("Do not propagate")

    @if_flying
    def has_received_give_feedback(self, msg):  # type: (Empty) -> None
        # rospy.loginfo("Got a gesture message")
        self.blocking_give_feedback()
        # rospy.loginfo("Gesture message callback done")

    def blocking_give_feedback(self):  # type: () -> bool
        if self.giving_feedback:
            return False
        if self.joy_set_teleop_mode:
            self.set_target_source(TargetMode.teleop, '')
            self.latest_source_time = rospy.Time.now()
            self.target_source_is_active = True
        self.hover()
        while self.state != State.hovering:
            rospy.sleep(0.1)
        self.giving_feedback = True
        self.give_feedback()
        self.giving_feedback = False
        return True

    @if_flying
    def has_received_hover(self, msg):  # type: (Empty) -> None
        self.hover()

    @if_flying
    def has_received_joy(self, msg):   # type: (Joy) -> None
        if not msg.buttons[self.deadman_button]:
            return

        if self.enforce_fence and not self.localized:
            return

        if self.target_mode != TargetMode.teleop and self.joy_set_teleop_mode:
            self.set_target_source(TargetMode.teleop, '')

        values = [msg.axes[axis] for axis in self.joy_axes]

        self.latest_source_time = rospy.Time.now()
        self.target_source_is_active = True

        if all([abs(v) < 0.01 for v in values]):
            self.hover()
            return

        if self.teleop_mode == TeleopMode.cmd:
            # the original bebop cmd

            horizontal_acceleration, vertical_speed, angular_speed = self.from_cmd(*values)
            self.target_acceleration = self.rotate_teleop(horizontal_acceleration)
            self.target_velocity = [0, 0, vertical_speed]
            if self.effective_angular_target_mode == TargetAngleMode.teleop:
                self.target_angular_speed = angular_speed
        else:
            self.target_acceleration = None
            self.target_velocity = self.rotate_teleop(
                [self.maximal_speed * values[0], self.maximal_speed * values[1],
                 self.maximal_vertical_speed * values[2]])
            if self.effective_angular_target_mode == TargetAngleMode.teleop:
                self.target_angular_speed = self.maximal_angular_speed * values[3]
        self.update_control()

    def rotate_teleop(self, vec):  # type: (np.ndarray) -> np.ndarray
        if self.teleop_frame == TeleopFrame.body:
            return rotate(vec, self.q, inverse=True)
        if self.teleop_frame == TeleopFrame.head:
            #  TODO: ignore head
            transform = get_transform(self.tf_buffer, self.frame_id, self.head_frame_id)
            if transform:
                vec = rotate(vec, transform[1], inverse=True)
            else:
                rospy.logwarn("Has received target cmd but cannot transform from %s to %s",
                              self.frame_id, self.head_frame_id)
                return
        else:
            return vec

    @if_flying
    def has_received_target_cmd(self, msg):  # type: (Twist) -> None
        if not (self.target_mode == TargetMode.cmd or
                self.effective_angular_target_mode == TargetAngleMode.cmd):
            return
        h_acc, vert_speed, ang_speed = self.from_cmd(
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.target_mode == TargetMode.cmd:
            self.target_acceleration = rotate(h_acc, self.q, inverse=True)
            self.target_velocity = [0, 0, vert_speed]
            self.latest_source_time = rospy.Time.now()
            self.target_source_is_active = True

        if self.effective_angular_target_mode == TargetAngleMode.cmd:
            self.target_angular_speed = ang_speed

    @if_flying
    def has_received_target(self, msg):  # type: (PoseStamped) -> None
        # rospy.loginfo("has_received_target")
        target_pose = pose_in_frame(self.tf_buffer, msg, self.frame_id)
        if not target_pose:
            rospy.logwarn("Has received target pose but cannot transform %s to %s",
                          msg, self.frame_id)
            return
        # else:
        #     rospy.logwarn("Has received target pose")

        _p = target_pose.pose.position
        _p = np.array([_p.x, _p.y, _p.z])

        if self.effective_angular_target_mode == TargetAngleMode.target_point and self.localized:
            self.target_yaw = target_yaw_to_observe(self.position, _p)
            self.target_angular_speed = None
            # rospy.loginfo("target_yaw %s, yaw %s", self.target_yaw, self.yaw)

        if self.target_mode != TargetMode.pos:
            return

        self.latest_source_time = rospy.Time.now()
        self.target_source_is_active = True
        # rospy.loginfo("target_source_is_active => active")

        self.target_position = _p

        if self.effective_angular_target_mode == TargetAngleMode.target_orientation:
            _o = target_pose.pose.orientation
            _, _, self.target_yaw = euler_from_quaternion([_o.x, _o.y, _o.z, _o.w])
            self.target_angular_speed = None

    def has_received_target_vel_in_world_frame(self, vel, omega
                                               ):  # type: (np.ndarray, float) -> None
        self.latest_source_time = rospy.Time.now()
        self.target_source_is_active = True
        self.target_velocity = np.array(vel)
        if self.effective_angular_target_mode == TargetAngleMode.vel:
            self.target_angular_speed = omega
            self.target_yaw = None

    def has_received_target_body_vel(self, msg):  # type: (Twist) -> None

        if self.state not in [State.flying, State.hovering]:
            return

        if self.target_mode != TargetMode.vel or not self.localized:
            return
        # convert in world frame
        vel = [msg.linear.x, msg.linear.y, msg.linear.z]
        ang_vel = [msg.angular.x, msg.angular.y, msg.angular.z]
        vel_world = rotate(vel, self.q, inverse=True)
        omega_world = rotate(ang_vel, self.q, inverse=True)[2]
        self.has_received_target_vel_in_world_frame(vel_world, omega_world)

    @if_flying
    def has_received_target_vel(self, msg):   # type: (TwistStamped) -> None
        if self.target_mode != TargetMode.vel:
            return
        # convert in world frame
        twist_s = twist_in_frame(self.tf_buffer, msg, self.frame_id)

        if not twist_s:
            rospy.logwarn("Has received target vel but cannot transform %s to %s",
                          msg, self.frame_id)
            return

        v = twist_s.twist.linear
        vel_world = [v.x, v.y, v.z]
        omega_world = twist_s.twist.angular.z
        self.has_received_target_vel_in_world_frame(vel_world, omega_world)

    @if_flying
    def has_received_target_odom(self, msg):  # type: (Odometry) -> None
        odom = odometry_in_frame(self.tf_buffer, msg, self.frame_id, self.frame_id)
        if not odom:
            rospy.logwarn("Has received target odom but cannot transform %s to %s",
                          msg, self.frame_id)
            return

        _p = odom.pose.pose.position
        _p = np.array([_p.x, _p.y, _p.z])
        if self.effective_angular_target_mode == TargetAngleMode.target_point and self.localized:
            self.target_yaw = target_yaw_to_observe(self.position, _p)
            self.target_angular_speed = None

        if self.target_mode != TargetMode.odom:
            return

        self.latest_source_time = rospy.Time.now()
        self.target_source_is_active = True

        _o = odom.pose.pose.orientation
        _v = odom.twist.twist.linear
        v = [_v.x, _v.y, 0]
        q = [_o.x, _o.y, _o.z, _o.w]
        _, _, yaw = euler_from_quaternion(q)
        q = quaternion_from_euler(0, 0, yaw + self.target_odom_yaw)

        if self.target_odom_z_is_relative:
            f = [self.target_odom_r, 0, self.target_odom_rel_z]
        else:
            f = [self.target_odom_r, 0, -_p[2] + self.target_odom_z]
        f = np.array(rotate(f, q, inverse=True))
        self.target_position = _p + f
        self.target_velocity = v

    def has_received_odometry(self, msg):  # type: (Odometry) -> None
        if not self.localization_active:
            return
        # Transform pose to World and twist to world
        odom = odometry_in_frame(self.tf_buffer, msg, self.frame_id, self.frame_id)
        if not odom:
            rospy.logwarn("Could not transform frame")
            return
        msg = odom
        self.last_localization = msg.header.stamp
        _p = msg.pose.pose.position
        self.z = _p.z
        _v = msg.twist.twist.linear
        o = msg.pose.pose.orientation
        # position in world_frame
        self.position = [_p.x, _p.y, _p.z]
        self.inside_fence = inside_fence_margin(self.position, self.pos_bounds, self.fence_margin)
        self.pitch, self.roll, self.yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.q = quaternion_from_euler(0, 0, self.yaw)

        # velocity in world frame
        self.velocity = [_v.x, _v.y, _v.z]
        self.localized = True
        self.publish_location()

# ----------------- Land/Takeoff

    def blocking_land(self):  # type: () -> bool
        if self.state not in [State.flying, State.hovering]:
            return False
        while self.state != State.landing:
            if self.state == State.landed:
                return True
            self.land()
            rospy.sleep(0.1)
        while self.state != State.landed:
            rospy.sleep(0.1)
        return True

    def blocking_takeoff(self):  # type: () -> bool
        if self.state != State.landed:
            return False
        while self.state != State.taking_off:
            self.safe_takeoff()
            rospy.sleep(0.1)
        while self.state != State.hovering:
            rospy.sleep(0.1)
        return True

    @if_flying
    def go_home_and_land(self):  # type: () -> None
        rospy.loginfo("Go home and land")
        if self.home is None or self.go_to_pose_no_feedback(self.home):
            self.blocking_land()

    def safe_takeoff(self):  # type: () -> None
        if self.state != State.landed:
            rospy.logwarn("Not allowed to take off (not landed)")
            return
        if self.enforce_fence and not self.inside_fence:
            rospy.logwarn("Not allowed to take off (not inside fence)")
            return
        if self.battery_state != BatteryState.ok:
            rospy.logwarn("Not allowed to take off (battery level is critical)")
            return
        if not self.can_fly():
            rospy.logwarn("Not allowed to take off (unsafe)")
            return
        self.reset_target()
        self.takeoff()

    def has_received_takeoff(self, msg):  # type: (Empty) -> None
        rospy.loginfo("Received takeoff")
        self.safe_takeoff()

    def has_received_safe_land(self, msg):  # type: (Empty) -> None
        rospy.loginfo("Received safe land")
        self.go_home_and_land()

    @staticmethod
    def cmd_from(acc, vert_vel, angular_speed):  # type: (np.ndarray, float, float) -> Twist
        return Twist()

    @staticmethod
    def from_cmd(
        pitch, roll, z, angular
    ):  # type: (float, float, float, float) -> Tuple[Tuple[float, float, float], float, float]
        return ((0, 0, 0), 0, 0)

# ----------------- Meta

    @abstractmethod
    def land(self):  # type: () -> None
        pass

    @abstractmethod
    def takeoff(self):  # type: () -> None
        pass

    @abstractmethod
    def hover(self):  # type: () -> None
        pass

    @abstractmethod
    def give_feedback(self):  # type: () -> None
        pass

    @abstractmethod
    def stop(self, msg=None):  # type: (Optional[Empty]) -> None
        pass

# ----------------- TODO: Observe
