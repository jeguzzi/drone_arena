import enum
import math
import struct
from collections import deque
from threading import RLock as Lock
from typing import Any, List, Optional, Tuple  # noqa

import numpy as np
from tf.transformations import euler_from_quaternion

import rospy
from crazyflie_driver.msg import FlightState, Hover, Position
from crazyflie_driver.srv import UpdateParams
from drone_arena_msgs.msg import BlinkMSequence  # BlinkMScript
from drone_arena_msgs.srv import SetXY, SetXYRequest, SetXYResponse  # noqa
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Quaternion, Twist  # noqa
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState as BatteryStateMsg
from std_msgs.msg import ColorRGBA, Empty, Int8, UInt8, Bool

from .control import BatteryState, Controller, State, button, if_flying, TargetMode

TAU = 0.02
ANGLE_TOL = 0.2
TOL = 0.1
RESET_TOL = 0.05
RESET_TIMEOUT = 2
THRUST_BUFFER_LENGTH = 3  # type: int

# 1 s if odom is published (as default) at 20 Hz
VZ_BUFFER_LENGTH = 20  # type: int
MIN_VZ_TO_STOP = -0.01


class HoverType(enum.Enum):
    position = 0
    velocity = 1


def to_ubyte(i):
    # type: (int) -> int
    return min(255, max(i, 0))


def to_sbyte(i):
    # type: (int) -> int
    return min(127, max(i, -127))


class CFController(Controller):

    def __init__(self):
        # type: () -> None
        super(CFController, self).__init__()
        self.led_enabled = None  # type: Optional[bool]
        self.led_mode = None  # type: Optional[int]
        self.last_state_update = None  # type: Optional[rospy.Time]
        self.cf_can_fly = False
        self.thrust_buffer = deque([], maxlen=THRUST_BUFFER_LENGTH)  # type: deque
        self.lock = Lock()  # type: Lock
        self.param_lock = Lock()  # type: Lock
        self.state_estimate = None  # type: Optional[Odometry]
        self.cf_z_tau = rospy.get_param('cf_z_tau', 2.0)  # type: float
        self.cf_vz_deadband = rospy.get_param('cf_vz_deadband', 1e-2)  # type: float
        self.hover_timer = None  # type: Optional[rospy.Timer]
        self.hover_distance = None  # type: Optional[float]
        self.hover_target_position = None  # type: Optional[np.ndarray]
        self.hover_target_yaw = None  # type: Optional[float]
        self.hover_takeoff_altitude = rospy.get_param('~takeoff_altitude', 0.5)
        self.hover_type = HoverType(rospy.get_param('~hover_type', 0))

        self.position_pub = rospy.Publisher('cmd_position', Position, queue_size=1)
        self.hover_pub = rospy.Publisher('cmd_hover', Hover, queue_size=1)
        self.cf_stop_pub = rospy.Publisher('cmd_stop', Empty, queue_size=1)

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy('update_params', UpdateParams)

        # Set if you want to use the mocap for state estimations. Not stricly needed as the fence
        # can be anyway defined in World and the tf be connected using the optitrack2odom node
        mocap_pose_topic = rospy.get_param('~mocap_pose', '')
        if mocap_pose_topic:
            self.use_mocap_for_state_estimation = True
            self.first_mocap_pose = True
            self.use_mocap_z = rospy.get_param('~use_mocap_z', True)
            rospy.Subscriber(mocap_pose_topic, PoseStamped, self.has_updated_mocap_pose,
                             queue_size=1)
            self.external_position_pub = rospy.Publisher("external_position", PointStamped,
                                                         queue_size=1)
        else:
            self.use_mocap_for_state_estimation = False

        self.land_at_altitude = rospy.get_param('~land_at_altitude', False)
        if self.land_at_altitude:
            self.vzs_length = rospy.get_param('land_at_altitude_buffer_length', VZ_BUFFER_LENGTH)
            self.vzs = deque([], maxlen=self.vzs_length)  # type: deque
            self.min_vz_to_stop = rospy.get_param(
                'land_at_altitude_min_vertical_speed', MIN_VZ_TO_STOP)

        rospy.Subscriber('state', FlightState, self.update_state, queue_size=1)
        rospy.Subscriber('reset', Empty, self.stop, queue_size=1)
        rospy.Subscriber('land', Empty, button(self.cf_land), queue_size=1)
        rospy.Subscriber('cf_odom', Odometry, self.update_odometry, queue_size=1)
        rospy.Subscriber('battery', BatteryStateMsg, self.update_battery, queue_size=1)
        rospy.Subscriber('set_pose', Pose, button(self.has_updated_pose), queue_size=1)
        rospy.Subscriber('led', ColorRGBA, self.has_updated_led, queue_size=1)
        rospy.Subscriber('led/mode', Int8, self.has_updated_led_mode, queue_size=1)
        # rospy.Subscriber('led/script', BlinkMScript, self.has_updated_led_script, queue_size=1)
        rospy.Subscriber('led/sequence', BlinkMSequence, self.has_updated_led_sequence,
                         queue_size=1)
        rospy.Subscriber('blinkM/brightness', UInt8, self.has_updated_led_brightness, 'blinkM',
                         queue_size=1)
        rospy.Subscriber('ring/brightness', UInt8, self.has_updated_led_brightness, 'ring',
                         queue_size=1)
        rospy.Subscriber('blinkM/fade_speed', UInt8, self.has_updated_led_fade_speed, queue_size=1)

        rospy.Subscriber('sound', UInt8, self.has_updated_sound, queue_size=1)

        rospy.Subscriber('autonomous', Bool, self.autonomy_has_changed, queue_size=1)

        rospy.Service('set_xy', SetXY, self.set_position_service)
        rospy.loginfo('Started set position service')

    def autonomy_has_changed(self, msg):
        # type: (Bool) -> None
        if msg.data and self.is_flying:
            self.stop_hovering()
        if msg.data:
            self.state = State.flying_autonomously
            self.target_mode = TargetMode.autonomous
        else:
            if self.is_flying:
                self.hover()

    def has_updated_sound(self, msg):
        # type: (UInt8) -> None
        # 0: off
        # 1: factory_test, repeated
        # 2: usb_connect
        # 3: usb_disconnect
        # 4: chg_done
        # 5: lowbatt, repeated
        # 6: startup
        # 7: calibrated
        # 8: range_slow, repeated
        # 9: range_fast, repeated
        # 10: starwars, repeated
        # 11: valkyries, repeated
        # 12: bypass, repeated
        # 13: siren, repeated
        # 14: tilt, repeated

        i = msg.data
        # TODO: read neffect from params sound/neffect
        if i > 14:
            i = 0

        with self.param_lock:
            param = 'sound/effect'
            rospy.set_param(param, i)
            try:
                self.update_params([param])
            except rospy.ServiceException:
                pass

    def has_updated_mocap_pose(self, pose):
        # type: (PoseStamped) -> None
        position = pose.pose.position
        if self.first_mocap_pose:
            if self.set_position(position.x, position.y,
                                 position.z if self.use_mocap_z else None,
                                 orientation=pose.pose.orientation):
                self.first_mocap_pose = False
            return
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()

        msg.point.x = position.x
        msg.point.y = position.y
        if self.use_mocap_z:
            msg.point.z = position.z
        elif self.state_estimate:
            msg.point.z = self.state_estimate.pose.pose.position.y
        else:
            return
        self.external_position_pub.publish(msg)

    def set_led_brightness(self, name, value):
        # type: (str, int) -> None
        with self.param_lock:
            param = '{0}/brightness'.format(name)
            rospy.set_param(param, value)
            try:
                self.update_params([param])
            except rospy.ServiceException:
                pass

    def set_led(self, red, green, blue):
        # type: (int, int, int) -> None

        self.set_led_sequence(colors=[(red, green, blue)], periods=[0], repetitions=0)

        # with self.param_lock:
        #     if self.led_mode == 0:
        #         self.set_led_mode(1)
        #     params = ["blinkM/solidRed1", "blinkM/solidGreen1", "blinkM/solidBlue1"]
        #     for param, color in zip(params, (red, green, blue)):
        #         rospy.set_param(param, color)
        #     try:
        #         self.update_params(params)
        #     except rospy.ServiceException:
        #         pass

    def set_led_mode(self, mode):
        # type: (int) -> None
        with self.param_lock:
            param = 'led/mode'
            rospy.set_param(param, mode)
            try:
                self.update_params([param])
                self.led_mode = mode
            except rospy.ServiceException:
                pass

    def set_led_script(self, id, repetitions=1, speed=0):
        # type: (int, int, int) -> None
        with self.param_lock:
            param = 'blinkM/script'
            data = struct.pack('BBbB', to_ubyte(id), to_ubyte(repetitions), to_sbyte(speed), 0)
            value, = struct.unpack('I', data)
            rospy.loginfo("Set blinkM script %d", value)
            rospy.set_param(param, value)
            try:
                self.update_params([param])
            except rospy.ServiceException:
                pass

    def set_led_fade_speed(self, speed):
        # type: (int) -> None
        with self.param_lock:
            param = 'blinkM/fadeSpeed'
            rospy.set_param(param, speed)
            try:
                self.update_params([param])
            except rospy.ServiceException:
                pass

    def has_updated_led(self, msg):
        # type: (ColorRGBA) -> None
        if self.led_enabled:
            rgb = [int(round(255 * color)) for color in (msg.r, msg.g, msg.b)]
            self.set_led(*rgb)

    def has_updated_led_mode(self, msg):
        # type: (Int8) -> None
        if self.led_enabled:
            self.set_led_mode(msg.data)

    # def has_updated_led_script(self, msg):
    #     # type: (BlinkMScript) -> None
    #     if self.led_enabled:
    #         self.set_led_script(msg.id, msg.repetitions, msg.speed)

    def set_led_sequence(self, colors, periods, repetitions):
        # type: (List[Tuple[int, int, int]], List[int], int) -> None
        with self.param_lock:
            param = 'led/sequence'
            periods += [0] * (3 - len(periods))
            data = struct.pack('BBBB', to_ubyte(periods[0]), to_ubyte(periods[1]),
                               to_ubyte(periods[2]), to_ubyte(repetitions))
            value, = struct.unpack('<i', data)
            rospy.set_param(param, value)
            params = [param]
            for i, (c, p) in list(enumerate(zip(colors, periods)))[:3]:
                if i > 0 and not p:
                    break
                param = 'led/color%d' % (i + 1)
                data = struct.pack('BBBB', to_ubyte(c[0]), to_ubyte(c[1]), to_ubyte(c[2]), 0)
                value, = struct.unpack('<i', data)
                rospy.set_param(param, value)
                params.append(param)
            try:
                self.update_params(params)
            except rospy.ServiceException:
                pass
            if self.led_mode == 0:
                self.set_led_mode(1)
        pass

    def has_updated_led_sequence(self, msg):
        # type: (BlinkMSequence) -> None
        if self.led_enabled:
            periods = [round(20 * t.to_sec()) for t in msg.periods]
            colors = [(math.floor(c.r * 255), math.floor(c.g * 255), math.floor(c.b * 255))
                      for c in msg.colors]
            self.set_led_sequence(colors=colors, periods=periods, repetitions=msg.repetitions)

    def has_updated_led_fade_speed(self, msg):
        # type: (Int8) -> None
        if self.led_enabled:
            self.set_led_fade_speed(msg.data)

    def has_updated_led_brightness(self, msg, name):
        # type: (Int8, str) -> None
        if self.led_enabled:
            self.set_led_brightness(name, msg.data)

    def set_position_service(self, req):
        # type: (SetXYRequest) -> SetXYResponse
        with self.lock:
            restart_hovering = False
            if self.hover_timer and self.hover_type == HoverType.position:
                restart_hovering = True
                self.stop_hovering()
                self.start_hovering_vel()
            r = self.set_position(x=req.x, y=req.y)
            if restart_hovering:
                self.stop_hovering()
                self.start_hovering()
            res = SetXYResponse()
            res.success = r
            return res

    def set_position(self, x, y, z=None, orientation=None):
        # type: (float, float, Optional[float], Optional[Quaternion]) -> bool
        if not self.state_estimate:
            return False
        if not self.reset_position(x, y, z=z, orientation=orientation):
            return False
        t = rospy.Time.now()
        while (rospy.Time.now() - t).to_sec() < RESET_TIMEOUT:
            cx = self.state_estimate.pose.pose.position.x
            cy = self.state_estimate.pose.pose.position.y
            has_converged = abs(x - cx) < RESET_TOL and abs(y - cy) < RESET_TOL
            if z is not None:
                cz = self.state_estimate.pose.pose.position.z
                has_converged = has_converged and abs(z - cz) < RESET_TOL
            if has_converged:
                rospy.loginfo("Position updated")
                return True
            rospy.sleep(0.1)
        return False

    def update_pose(self, pose, reset=True):
        # type: (Pose, bool) -> bool
        if self.use_mocap_for_state_estimation:
            self.first_mocap_pose = True
            return True
        return self.set_position(x=pose.position.x, y=pose.position.y)

    # def update_pose(self, pose, reset=False):
        # # type: (Pose, bool) -> bool
        # if not self.state_estimate:
        #     rospy.logwarn("will not update pose: no state estimation")
        #     return False
        # rospy.loginfo("will update pose")
        # if reset:
        #     self.reset(pose)
        # msg = PointStamped()
        # msg.header.stamp = rospy.Time.now()
        # tx = pose.position.x
        # ty = pose.position.y
        # msg.point.x = tx
        # msg.point.y = ty
        # msg.point.z = self.state_estimate.pose.position.z
        # self.external_pose_pub.publish(msg)
        # t = rospy.Time.now()
        # while (rospy.Time.now() - t).to_sec() < RESET_TIMEOUT:
        #     x = self.state_estimate.pose.position.x
        #     y = self.state_estimate.pose.position.y
        #     if (abs(x - tx) < RESET_TOL and abs(y - ty) < RESET_TOL):
        #         rospy.loginfo("Pose updated")
        #         return True
        #     rospy.sleep(0.1)
        # rospy.logwarn("Pose not updated")
        # return False

    def reset_position(self, x, y, z=None, orientation=None):
        # type: (float, float, Optional[float], Optional[Quaternion]) -> bool
        if z is None:
            if not self.state_estimate:
                return False
            else:
                z = self.state_estimate.pose.pose.position.z
        if orientation is None:
            if not self.state_estimate:
                return False
            else:
                orientation = self.state_estimate.pose.pose.orientation
        with self.param_lock:
            rospy.loginfo("Reset Kalman filter")
            rospy.set_param("kalman/initialX", x)
            rospy.set_param("kalman/initialY", y)
            rospy.set_param("kalman/initialZ", z)
            params = ["kalman/initialX", "kalman/initialY", "kalman/initialZ"]

            # rospy.set_param("kalman/initialQx", orientation.x)
            # rospy.set_param("kalman/initialQy", orientation.y)
            # rospy.set_param("kalman/initialQz", orientation.z)
            # rospy.set_param("kalman/initialQw", orientation.w)
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(q)
            rospy.set_param("kalman/initialYaw", yaw)

            rospy.set_param("kalman/resetEstimation", 1)
            # params += ["kalman/initialQx", "kalman/initialQy", "kalman/initialQz",
            #            "kalman/initialQw"]

            params.append("kalman/initialYaw")

            try:
                self.update_params(params)
                self.update_params(["kalman/resetEstimation"])
            except rospy.ServiceException:
                return False

            rospy.loginfo("Reset pose to (%s, %s, %s), (%s, %s, %s, %s)",
                          x, y, z, orientation.x, orientation.y, orientation.z, orientation.w)

            return True

    # def reset(self, pose):
    #     # type: (Pose) -> bool
    #     return self.reset_position(x=pose.position.x, y=pose.position.y)

    def has_updated_pose(self, msg):
        # TODO: allow only if landed or hovering
        # type: (Pose) -> None
        with self.lock:
            restart_hovering = False
            if self.hover_timer and self.hover_type == HoverType.position:
                restart_hovering = True
                self.stop_hovering()
                self.start_hovering_vel()
            self.update_pose(msg, reset=True)
            if restart_hovering:
                self.stop_hovering()
                self.start_hovering()
        # if self.state == State.hovering:
        #     self.hover_target[0] = msg.position.x
        #     self.hover_target[1] = msg.position.y

    def update_battery(self, msg):
        # type: (BatteryStateMsg) -> None
        self.battery_percent = msg.percentage * 100

    # ---- concrete implementation

    # TODO: parametrize
    def give_feedback(self):
        # type: () -> None
        if self.state_estimate is None:
            rospy.logwarn("No state estimate")
            return
        # t = 0.0
        A = rospy.get_param('~feedback/amplitude', 1)
        n = rospy.get_param('~feedback/movements', 1)
        t_up = rospy.get_param('~feedback/up', 0.15)
        t_down = rospy.get_param('~feedback/down', 0.5)
        # T = T = 2 * math.pi * n / omega
        # omega = 6.0
        # T = 2 * math.pi * n / omega
        # dt = 0.1

        msg = Hover()
        msg.vx = 0
        msg.vy = 0
        msg.yawrate = 0
        z = self.state_estimate.pose.pose.position.z
        with self.lock:
            rospy.loginfo("Start gesture with A %s, n %s, dt (%s, %s)", A, n, t_up, t_down)
            for _ in range(n):
                for dz, dt in zip([A, 0], [t_up, t_down]):
                    msg.zDistance = z + dz
                    msg.header.stamp = rospy.Time.now()
                    # rospy.loginfo("Send hover cmd %s", msg)
                    self.hover_pub.publish(msg)
                    rospy.sleep(dt)
        # rospy.loginfo("Gesture done")

        # while t < T:
        #     dz = A * math.sin(t * omega)
        #     msg.zDistance = z + dz
        #     msg.header.stamp = rospy.Time.now()
        #     self.hover_pub.publish(msg)
        #     t += dt
        #     rospy.sleep(dt)

    # def give_feedback(self):
    #     with self.lock:
    #         for _ in range(2):
    #             self.publish_target_body_vel([0, 0, self.maximal_angular_speed], 0)
    #             rospy.sleep(0.2)
    #             self.publish_target_body_vel([0, 0, -self.maximal_angular_speed], 0)
    #             rospy.sleep(0.2)
    #         self.publish_target_body_vel([0, 0, 0], 0)

    def stop_hovering(self):
        # type: () -> None
        with self.lock:
            if self.hover_timer:
                self.hover_timer.shutdown()
                self.hover_timer = None
                rospy.loginfo("Stop hovering")

    def publish_target_body_vel(self, velocity, angular_speed):
        # type: (np.ndarray, float) -> None
        # rospy.loginfo("Publish body vel %s %s", velocity, angular_speed)
        self.stop_hovering()
        super(CFController, self).publish_target_body_vel(velocity, angular_speed)

        if self.state_estimate:
            z = self.state_estimate.pose.pose.position.z
        else:
            rospy.logwarn("No state estimate")
            return

        if abs(velocity[2]) <= self.cf_vz_deadband:
            # fly flat, i.e. send a constant msg.zDistance
            # Remember the first altitude as hover_distance
            # TODO: not very safe
            if self.hover_distance is None:
                self.hover_distance = z
            z = self.hover_distance
        else:
            self.hover_distance = None

        msg = Hover()
        msg.header.stamp = rospy.Time.now()
        msg.vx = velocity[0]
        msg.vy = velocity[1]
        msg.yawrate = -180 * angular_speed / np.pi
        msg.zDistance = z + velocity[2] / self.cf_z_tau
        self.hover_pub.publish(msg)
        self.state = State.flying
        self.hover_target_position = self.hover_target_yaw = None

    def publish_target(self, des_target, des_target_yaw, hovering=False):
        # type: (np.ndarray, float, bool) -> None
        # rospy.loginfo("Publish target %s %s (%s)", des_target, des_target_yaw, hovering)
        if not hovering:
            self.stop_hovering()
            self.state = State.flying
        super(CFController, self).publish_target(des_target, des_target_yaw)
        msg = Position()
        msg.header.stamp = rospy.Time.now()
        msg.x, msg.y, msg.z = des_target
        msg.yaw = 180 * des_target_yaw / np.pi
        self.position_pub.publish(msg)
        self.hover_target_position = des_target
        self.hover_target_yaw = des_target_yaw
        self.hover_distance = None

    def publish_target_cmd(self, acc, vert_vel, angular_speed):
        # type: (np.ndarray, float, float) -> None
        rospy.error('publish_target_cmd not implemented')
        pass

    def start_hovering_vel(self, delta=[0, 0, 0]):
        # type: (np.ndarray) -> None
        if not self.state_estimate:
            return

        msg = Hover()
        msg.vx = 0
        msg.vy = 0
        msg.yawrate = 0
        msg.zDistance = self.state_estimate.pose.pose.position.z + delta[2]
        self.hover_target_position = [None, None, msg.zDistance]

        def callback(event):
            # type: (rospy.Timer) -> None
            msg.header.stamp = rospy.Time.now()
            self.hover_pub.publish(msg)
        self.hover_pub.publish(msg)
        self.hover_timer = rospy.Timer(rospy.Duration(0.25), callback, oneshot=False)

    def start_hovering(self, delta=[0, 0, 0]):
        # type: (np.ndarray) -> None
        if not self.state_estimate:
            return
        p = self.state_estimate.pose.pose.position
        q = self.state_estimate.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        hover_target = np.array([p.x, p.y, p.z]) + delta

        rospy.loginfo("Start hovering at %s", hover_target)

        def callback(event):
            # type: (rospy.Timer) -> None
            if not self.giving_feedback:
                self.publish_target(hover_target, yaw, hovering=True)
        self.publish_target(hover_target, yaw, hovering=True)

        self.hover_timer = rospy.Timer(rospy.Duration(0.25), callback, oneshot=False)

    def hover(self, delta=[0, 0, 0], _type=None, state=State.hovering):
        # type: (np.ndarray, Optional[HoverType], State) -> None
        if not self.state_estimate:
            rospy.logwarn("No state estimate")
            return
        with self.lock:
            if self.state == state:
                return
            self.stop_hovering()
            if _type is None:
                _type = self.hover_type
            if _type == HoverType.position:
                self.start_hovering(delta=delta)
            else:
                self.start_hovering_vel(delta=delta)
            self.hover_distance = None
            self.state = state

    def has_connected(self):
        # type: () -> None
        self.led_enabled = (rospy.get_param('blinkM/isInit', False) or
                            rospy.get_param('ring/isInit', False))
        rospy.loginfo("Led is enabled? %d", self.led_enabled)
        self.led_mode = rospy.get_param('led/mode', None)
        self.battery_state = None
        self.cf_can_fly = False
        self.thrust_buffer.clear()
        self.state_estimate = None
        self.first_mocap_pose = True

    def update_state(self, msg):
        # type: (FlightState) -> None
        if self.last_state_update is None:
            self.has_connected()
        self.last_state_update = rospy.Time.now()
        self.thrust_buffer.append(msg.thrust)
        if self.battery_percent is not None:
            if self.battery_state is None:
                self.battery_state = BatteryState.ok
            if self.battery_state == BatteryState.critical:
                if not msg.battery_low and self.battery_percent > 0.15:
                    self.battery_state = BatteryState.ok
            else:
                # HACK
                # if msg.battery_low or self.battery_percent < 0.05:
                if msg.battery_low or self.battery_percent < -0.05:
                    rospy.loginfo("update_state => battery critical")
                    self.battery_state = BatteryState.critical
        if self.state not in [State.landed, State.taking_off] and not any(self.thrust_buffer):
            rospy.loginfo("update_state %s, thrust = 0 for 3 seconds =>  landed", self.state)
            self.stop()
        self.cf_can_fly = msg.can_fly

    def can_fly(self):
        # type: () -> bool
        return super(CFController, self).can_fly() and self.cf_can_fly

    def soft_land(self, max_thrust=60000):
        # type: (int) -> None
        self.stop_hovering()
        if not self.thrust_buffer or self.thrust_buffer[-1] < 0.6:
            self.stop()
            return
        thrust = self.thrust_buffer[-1] / max_thrust
        print("Start by thrust %.2f\n" % thrust)
        for i in range(1, 10):
            s_thrust = 0.6 * i / 10 + thrust * (1 - i / 10)
            # s_thrust = thrust
            # print("-> thrust %.2f\n" % s_thrust)
            self.set_thrust(s_thrust)
            rospy.sleep(0.1)
        self.stop()

    def update(self, evt):
        # type: (rospy.TimerEvent) -> None
        super(CFController, self).update(evt)

        # TODO: param for timeout (state timeout), now state is send out at 1 Hz
        if self.state != State.landed:
            if self.last_state_update is not None:
                dt = (rospy.Time.now() - self.last_state_update).to_sec()
                if dt > 3:
                    # is disconnected (maybe shutdown)
                    rospy.logwarn("lost connection => stop and set to landed")
                    self.stop()
                    self.battery_state = None
                    self.last_state_update = None

        if self.state in [State.landing] and self.state_estimate is not None:
            z = self.state_estimate.pose.pose.position.z
            if z < 0.05:
                # self.soft_land()
                self.stop()
            if self.land_at_altitude:
                self.vzs.append(self.state_estimate.twist.twist.linear.z)
                if len(self.vzs) == self.vzs_length and max(self.vzs) > self.min_vz_to_stop:
                    self.stop()
        if self.state in [State.taking_off, State.flying]:
            if self.near_target():
                rospy.loginfo("update_state => is hovering")
                self.state = State.hovering

    def near_target(self):
        # type: () -> bool
        if self.hover_target_position is not None and self.state_estimate is not None:
            p = self.state_estimate.pose.pose.position
            # rospy.loginfo("near target? %s %s", self.target_position, [p.x, p.y, p.z])
            if self.hover_type == HoverType.position:
                dist = np.linalg.norm(
                    np.array(self.hover_target_position) - np.array([p.x, p.y, p.z]))
                rospy.loginfo('dist1 %s', dist)
            else:
                dist = abs(self.hover_target_position[2] - p.z)
                rospy.loginfo('dist2 %s', dist)
            if dist > TOL:
                return False
            if self.hover_target_yaw is not None:
                q = self.state_estimate.pose.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                return abs(np.unwrap([0, self.hover_target_yaw - yaw])[1]) < ANGLE_TOL
            else:
                return True
        return False

    @if_flying
    def cf_land(self, msg):
        # type: (Empty) -> None
        self.land()

    def land(self):
        # type: (Empty) -> None
        rospy.loginfo("Land")
        p = self.state_estimate.pose.pose.position
        self.hover(delta=[0, 0, -p.z + 0.05], state=State.landing)
        if self.land_at_altitude:
            self.vzs.clear()

    def set_thrust(self, value, max_thrust=60000):
        # type: (float, float) -> None
        # param value: percentage of maximal thrust
        msg = Twist()
        msg.linear.z = value * max_thrust
        self.des_cmd_pub.publish(msg)

    def takeoff(self):
        # type: (Empty) -> None
        with self.lock:
            if self.use_mocap_for_state_estimation:
                self.first_mocap_pose = True
                while self.first_mocap_pose:
                    rospy.sleep(0.1)
            rospy.loginfo("Takeoff")
            # we prespin to avoid shutting down NINA :-/
            for _ in range(10):
                self.set_thrust(0)
                rospy.sleep(0.01)
            for i in range(1, 20):
                self.set_thrust(i * 0.8 / 20)
                rospy.sleep(0.1)
            self.hover(delta=[0, 0, self.hover_takeoff_altitude], state=State.taking_off)

    def stop(self, msg=None):
        # type: (Optional[Empty]) -> None
        self.stop_hovering()
        self.cf_stop_pub.publish()
        self.state = State.landed
        self.hover_distance = None
        rospy.loginfo("Stop")

    def update_odometry(self, msg):
        # type: (Odometry) -> None
        self.state_estimate = msg
