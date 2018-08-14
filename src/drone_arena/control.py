from __future__ import division

import math

import enum
import numpy as np

import actionlib
import diagnostic_msgs
import diagnostic_updater
import rospy
import tf2_geometry_msgs
import tf2_ros
from drone_arena.cfg import ArenaConfig
from drone_arena.msg import GoToPoseAction, GoToPoseFeedback, GoToPoseResult
from drone_arena.temporized import Temporized
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import (Point, PoseStamped, Quaternion, Twist,
                               TwistStamped, Vector, Vector3Stamped)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Header, String
from tf.transformations import (euler_from_quaternion, quaternion_conjugate,
                                quaternion_from_euler, quaternion_multiply)

from .fence_control import angular_control, fence_control, inside_fence_margin


class TargetMode(enum.Enum):
    cmd = 0
    pos = 1
    vel = 2
    odom = 3


class TargetAngleMode(enum.Enum):
    cmd = 0
    point = 1
    vel = 2
    target = 3
    target_pose = 4
    plane = 5


class TeleopMode(enum.Enum):
    cmd = 0
    vel = 1


class TeleopFrame(enum.Enum):
    body = 0
    world = 1
    head = 2


class State(enum.Enum):
    landed = 0
    flying = 1
    hovering = 2


button = Temporized(1)


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


def target_yaw_to_observe(observer_point, target_point):
    d = np.array(target_point) - np.array(observer_point)
    return np.arctan2(d[1], d[0])


class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        super(Controller, self).__init__()
        rospy.init_node('fence_control', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._target_mode = TargetMode.cmd
        self.target_angle_mode = TargetAngleMode.cmd
        self.teleop_mode = TeleopMode.cmd
        self.teleop_frame = TeleopFrame.body
        self._state = State.landed

        self.target_position = None
        self.target_velocity = None
        self.target_acceleration = None
        self.target_yaw = None
        self.target_angular_speed = None

        self.publish_cmd_vel = True
        self.publish_body_vel = True
        self.publish_target = True
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)

        self.maximal_speed = rospy.get_param('max_speed', 0.5)
        self.maximal_angular_speed = rospy.get_param('max_angular_speed', 0.5)
        self.maximal_acceleration = rospy.get_param('max_acceleration', 1)

        self.frame_id = rospy.get_param('~frame_id', 'World')
        self.head_frame_id = rospy.get_param('~head_frame_id', 'head')

        self.control_timeout = rospy.get_param('~control_timeout', 0.2)
        self.latest_cmd_time = None

        self.srv = Server(ArenaConfig, self.callback)

        self.observe_point = None
        self.head_point = None

        rospy.Subscriber('takeoff/safe', Empty, button(self.has_received_takeoff))
        rospy.Subscriber('land', Empty, button(self.has_received_land))
        rospy.Subscriber('odom', Odometry, self.has_received_odometry)
        rospy.Subscriber('target', PoseStamped, self.has_received_target)
        rospy.Subscriber('target/body_vel', Twist, self.has_received_target_body_vel)
        rospy.Subscriber('target/vel', TwistStamped, self.has_received_target_vel)
        rospy.Subscriber('target/odom', Odometry, self.has_received_target_odom)
        # TODO: /head/mocap_odom
        rospy.Subscriber('target/cmd_vel', Twist, self.has_received_input_cmd)

        rospy.Subscriber('target/enable', Bool,
                         button(self.has_received_enable_tracking), callback_args='pos')
        rospy.Subscriber('target/vel/enable', Bool,
                         button(self.has_received_enable_tracking, callback_args='vel'))
        rospy.Subscriber('target/odom/enable', Bool,
                         button(self.has_received_enable_tracking, callback_args='odom'))

        self.enable_target_pub = rospy.Publisher(
            'target/enable', Bool, queue_size=1, latch=True)
        self.enable_vel_target_pub = rospy.Publisher(
            'target/vel/enable', Bool, queue_size=1, latch=True)
        self.enable_odom_target_pub = rospy.Publisher(
            'target/odom/enable', Bool, queue_size=1, latch=True)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)
        self.init_diagnostics()
        rospy.spin()

# --------------- dynamic reconfig

    def callback(self, config, level):
        self.eta = config['eta']
        self.tau = config['tau']
        self.rotation_tau = config['rotation_tau']
        self.delay = config['delay']
        self.maximal_acceleration = config['max_acceleration']
        self.maximal_speed = config['max_speed']
        self.maximal_angular_speed = config['max_angular_speed']

        self.enforce_fence = config['enable_fence']

        self.teleop_mode = TeleopMode(config['teleop_mode'])
        self.teleop_frame = TeleopFrame(config['teleop_frame'])

        self.target_odom_r = config['track_distance']
        self.target_odom_z_is_relative = config['track_vertical_relative']
        self.target_odom_z = config['track_vertical']
        self.target_odom_yaw = config['track_yaw']

        self.localization_timeout = config['localization_timeout']
        self.control_timeout = config['control_timeout']

        self.pos_tol = config['position_tol']
        self.angle_tol = config['angle_tol']

        return config

# --------------- odom (head) following

    def init_odom_following(self):
        self.target_odom_r = rospy.get_param('~track_distance', False)
        self.target_odom_z_is_relative = rospy.get_param('~track_vertical_head', False)
        self.target_odom_dz = rospy.get_param('~head_altitude_difference', -0.2)
        self.target_odom_z = rospy.get_param('~head_altitude', 1.5)

# --------------- Diagnostics

    def init_diagnostics(self):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("fence controller")
        self.updater.add("Localization", self.localization_diagnostics)
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)

    def update_diagnostics(self, event):
        self.updater.update()

    def localization_diagnostics(self, stat):
        if self.localized:
            if self.inside_fence:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Ok")
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Outside fence")
        else:
            stat.summary(
                diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Not localized")

# --------------- Localization

    @property
    def localized(self):
        return self._localized

    @localized.setter
    def localized(self, value):
        if self._localized != value:
            self._localized = value
            self.publish_location()
            self.handle_non_localized()

    def publish_location(self):
        if not self.localized:
            self.localization_pub.publish('')
        elif not self.inside_fence:
            self.localization_pub.publish('out')
        else:
            self.localization_pub.publish('in')

    def init_localization(self):
        self.localization_pub = rospy.Publisher('location', String, queue_size=1, latch=True)
        self._localized = None
        self.localized = False
        self.localization_active = True
        self.last_localization = None
        self.localization_timeout = rospy.get_param('~localization_timeout', 1)
        rospy.Subscriber('localization_active', Bool, self.has_received_localization_active)

    def has_received_localization_active(self, msg):
        self.localization_active = msg.data

    def update_localization_state(self):
        if self.localized:
            if (rospy.Time.now() - self.last_localization).to_sec() > self.localization_timeout:
                rospy.logwarn("No more localized")
                self.localized = False

# --------------- fence

    @property
    def max_height(self):
        return self.pos_bounds[2][1]

    def init_fence(self):
        self.fence = rospy.get_param("~pos_bounds", ((-1.8, 1.8), (-1.8, 1.8), (0.5, 2.0)))
        _home = rospy.get_param("~home", (0, 0, 1))
        self.home_p = _home
        self.home = PoseStamped()
        self.home.header.frame_id = self.frame_id
        self.home.pose.position.x = _home[0]
        self.home.pose.position.y = _home[1]
        self.home.pose.position.z = _home[2]
        self.home.pose.orientation.w = 1
        self.fence_margin = 0.5
        self.inside_fence = False

# --------------- Go to pose action

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

    def init_action_server(self):
        self.pos_tol = rospy.get_param('position_tol', 0.1)
        self.angle_tol = rospy.get_param('angle_tol', 0.2)

        rospy.loginfo('Will start SimpleActionServer fence_control')
        self._as = actionlib.SimpleActionServer(
            'fence_control', GoToPoseAction, execute_cb=self.execute_cb,
            auto_start=False)
        self._as.start()
        rospy.loginfo('Started SimpleActionServer fence_control')

# ----------------- Controller

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        if self._state != value:
            self._state = value
            if value == State.hovering:
                self.hover()

    def handle_non_localized(self):
        pass

    def update(self, evt):
        self.update_localization_state()
        self.update_cmd_timeout()
        if self.state not in [State.flying, State.hovering] or not self.localized:
            return
        if self.flying:
            self.update_control()

    def update_cmd_timeout(self):
        if self.flying and self.latest_cmd_time:
            delta = rospy.Time.now() - self.latest_cmd_time
            if delta.to_sec() > self.control_timeout:
                rospy.logwarn("No recent control => switch to hovering")
                self.state = State.hovering

# ----------------- Flying

    def update_control(self):
        if not self.localized:
            self.hover()  # TODO: self.pub_cmd.publish(Twist())
            return

        if not self.inside_fence:
            self.hover()  # TODO: better safety
            return

        des_target, des_velocity, des_acceleration = fence_control(
            self.position, self.velocity, self.target_position, self.target_velocity,
            self.target_acceleration, delay=self.delay, fence=self.fence,
            maximal_acceleration=self.maximal_acceleration, maximal_speed=self.maximal_speed,
            maximal_vertical_speed=self.maximal_vertical_speed, eta=self.eta, tau=self.tau)

        des_target_yaw, des_angular_speed = angular_control(
            self.yaw, self.target_yaw, self.target_angular_speed, rotation_tau=self.rotation_tau,
            maximal_angular_speed=self.max_angular_speed)

        if self.publish_target:  # Maybe just a Point (lets see what cf needs)
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.pose.position = Point(*des_target)
            msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, des_target_yaw))
            self.des_pose_pub.publish(msg)

        if self.publish_body_vel:
            vec = [self.des_velocity[0], self.des_velocity[1], self.des_velocity[2], 0]
            vec = rotate(vec, self.q, inverse=False)[:2]
            msg = Twist()
            msg.linear = Vector(*vec)
            msg.angular = Vector(0, 0, des_angular_speed)
            self.des_vel_pub.publish(msg)

        if self.publish_cmd:
            vec = [des_acceleration[0], des_acceleration[1], 0, 0]
            vec = rotate(vec, self.q, inverse=False)[:2]
            vec = self.cmd_from_acc(des_acceleration, des_velocity[2])
            msg = Twist()
            msg.linear = Vector(*vec)
            msg.angular = Vector(0, 0, des_angular_speed)
            self.des_cmd.publish(msg)

    @property
    def target_mode(self):
        return self._target_mode

    @target_mode.setter
    def target_mode(self, value):
        if value != self._target_mode:
            if self._target_mode == TargetMode.pos:
                self.enable_target_pub.publish(False)
            if self._target_mode == TargetMode.vel:
                self.enable_target_vel_pub.publish(False)
            if self._target_mode == TargetMode.odom:
                self.enable_target_odom_pub.publish(False)
            self._target_mode = value

    def has_received_enable_tracking(self, msg, mode):
        if self.target_mode == mode and not msg.value:
            self.target_mode = TargetMode.cmd
        if msg.value:
            self.target_mode = mode

    def has_received_target_cmd(self, msg):

        if not self.localized:
            return

        self.target_mode = TargetMode.cmd
        vec = [msg.linear, msg.linear.y, msg.linear.z]
        if self.teleop_frame == TeleopFrame.body:
            # TODO: ArenaConfig.Arena_World|Arena_Body|Arena_Head
            vec = rotate(vec, self.q, inverse=True)[:2]
        elif self.teleop_frame == TeleopFrame.world:
            transform = get_transform(self.tf_buffer, self.frame_id, self.head_frame_id)
            if transform:
                vec = rotate(vec, transform[1], inverse=True)[:2]
            else:
                rospy.logwarn("Has received target cmd but cannot transform from %s to %s",
                              self.frame_id, self.head_frame_id)
                return
        else:
            vec = vec[:2]

        self.latest_cmd_time = rospy.Time.now()

        if self.teleop_mode == TeleopMode.cmd:
            self.target_acceleration = self.acc_from_cmd(vec)
            self.target_velocity = [0, 0, self.vert_vel_from_cmd(vec[2])]
        else:
            self.target_velocity = self.vel_from_cmd(vec)
            self.target_acceleration = None

        if self.target_angle_mode == TargetAngleMode.cmd:
            self.target_angular_speed = self.angular_speed_from_cmd(msg.angular.z)

        self.update_control()

    def has_received_target(self, msg):
        target_pose = pose_in_frame(self.tf_buffer, msg, self.frame_id)
        if not target_pose:
            rospy.logwarn("Has received target pose but cannot transform %s to %s",
                          msg, self.frame_id)
            return

        _p = target_pose.pose.position
        _p = np.array([_p.x, _p.y, _p.z])

        if self.target_angle_mode == TargetAngleMode.target and self.localized:
            self.target_yaw = target_yaw_to_observe(self.position, _p)
            self.target_angular_speed = None

        if self.target_mode != TargetMode.pos:
            return

        self.latest_cmd_time = rospy.Time.now()

        self.target_position = _p
        self.target_velocity = np.array([0, 0, 0])

        if self.target_angle_mode == TargetAngleMode.target_pose:
            _o = target_pose.pose.orientation
            _, _, self.target_yaw = euler_from_quaternion([_o.x, _o.y, _o.z, _o.w])
            self.target_angular_speed = None

    def has_received_target_vel_in_world_frame(self, vel, omega):
        self.latest_cmd_time = rospy.Time.now()
        self.target_velocity = np.array(vel)
        self.target_position = None
        if self.target_angle_mode == TargetAngleMode.plane:
            self.target_yaw = np.arctan2(vel[1], vel[0])
            self.target_angular_speed = None
        if self.target_angle_mode == TargetAngleMode.vel:
            self.target_angular_speed = rotate(omega, self.q, inverse=True)[2]
            self.target_yaw = None

    def has_received_target_body_vel(self, msg):
        if self.target_mode != TargetMode.vel or not self.localized:
            return
        # convert in world frame
        vel = [msg.linear.x, msg.linear.y, msg.linear.z, 0]
        ang_vel = [msg.angular.x, msg.angular.y, msg.angular.z, 0]
        vel_world = rotate(vel, self.q, inverse=True)[:3]
        omega_world = rotate(ang_vel, self.q, inverse=True)[2]
        self.has_received_target_vel_in_world_frame(vel_world, omega_world)

    def has_received_target_vel(self, msg):
        if self.target_mode != TargetMode.vel:
            return
        # convert in world frame
        twist_s = twist_in_frame(self.tf_buffer, msg, self.frame_id)
        v = twist_s.twist.linear
        vel_world = [v.x, v.y, v.z]
        omega_world = twist_s.twist.angular.z
        self.has_received_target_vel_in_world_frame(vel_world, omega_world)

    def has_received_target_odom(self, msg):

        odom = odometry_in_frame(self.tf_buffer, msg, self.frame_id, self.frame_id)
        if not odom:
            rospy.logwarn("Has received target odom but cannot transform %s to %s",
                          msg, self.frame_id)
            return

        _p = odom.pose.pose.position
        _p = np.array([_p.x, _p.y, _p.z])
        if self.target_angle_mode == TargetAngleMode.target and self.localized:
            self.target_yaw = target_yaw_to_observe(self.position, _p)
            self.target_angular_yaw = None

        if self.target_mode != TargetMode.odom:
            return

        self.latest_cmd_time = rospy.Time.now()

        _o = odom.pose.pose.orientation
        _v = odom.twist.twist.linear
        v = [_v.x, _v.y, 0]
        q = [_o.x, _o.y, _o.z, _o.w]
        _, _, yaw = euler_from_quaternion(q)
        q = quaternion_from_euler(0, 0, yaw + self.target_odom_yaw)

        if self.target_odom_z_is_relative:
            f = [self.target_odom_r, 0, self.target_odom_dz]
        else:
            f = [self.target_odom_r, 0, -_p.z + self.target_odom_z]
        f = np.array(rotate(f, q, inverse=True)[:3])
        self.target_position = _p + f
        self.target_velocity = v

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
        self.inside_fence = inside_fence_margin(self.position, self.pos_bounds, self.fence_margin)
        self.q = [o.x, o.y, o.z, o.w]
        _, _, self.yaw = euler_from_quaternion(self.q)
        # velocity in world frame
        self.velocity = [_v.x, _v.y, _v.z]
        self.localized = True
        self.publish_location()

# ----------------- Land/Takeoff

    def has_received_land(self, msg):
        rospy.loginfo("Received landing")
        self.state = State.landed

    def has_received_takeoff(self, msg):
        rospy.loginfo("Received takeoff")
        if self.inside_fence:
            self.pub_takeoff.publish(Empty())
            self._state = State.hovering
        else:
            rospy.logwarn("Outside fence, will not takeoff")

# ----------------- TODO: Observe
