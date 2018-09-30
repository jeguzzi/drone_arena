import math
from threading import RLock as Lock

import numpy as np

import rospy
from crazyflie_driver.msg import FlightState, Hover, Position
from crazyflie_driver.srv import UpdateParams
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
from typing import Optional, Tuple  # noqa

from .control import BatteryState, Controller, State, button

TAU = 0.02
ANGLE_TOL = 0.2
TOL = 0.1


class CFController(Controller):

    def __init__(self):
        # type: () -> None
        super(CFController, self).__init__()
        self.last_state_update = None  # type: Optional[rospy.Time]
        self.cf_can_fly = False
        self.lock = Lock()  # type: Lock
        self.state_estimate = None  # type: Optional[Odometry]
        self.hover_timer = None  # type: Optional[rospy.Timer]
        self.hover_distance = None  # type: Optional[float]
        self.hover_target_position = None  # type: Optional[np.ndarray]
        self.hover_target_yaw = None  # type: Optional[float]
        self.hover_takeoff_altitude = rospy.get_param('~takeoff_altitude', 0.5)

        self.position_pub = rospy.Publisher('cmd_position', Position, queue_size=1)
        self.hover_pub = rospy.Publisher('cmd_hover', Hover, queue_size=1)
        self.stop_pub = rospy.Publisher('cmd_stop', Empty, queue_size=1)

        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.Subscriber('state', FlightState, self.update_state, queue_size=1)
        rospy.Subscriber('reset', Empty, self.stop, queue_size=1)
        rospy.Subscriber('land', Empty, button(self.cf_land), queue_size=1)
        rospy.Subscriber('cf_odom', Odometry, self.update_odometry, queue_size=1)

    # ---- concrete implementation

    # TODO: parametrize
    def give_feedback(self):
        # type: () -> None
        if self.state_estimate is None:
            rospy.logwarn("No state estimate")
            return
        z = self.state_estimate.pose.position.z
        t = 0.0
        A = 0.1
        n = 2
        omega = 6.0
        T = 2 * math.pi * n / omega
        dt = 0.1
        msg = Hover()
        msg.vx = 0
        msg.vy = 0
        msg.yawrate = 0
        while t < T:
            dz = A * math.sin(t * omega)
            msg.zDistance = z + dz
            msg.header.stamp = rospy.Time.now()
            self.hover_pub.publish(msg)
            t += dt
            rospy.sleep(dt)

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
        if self.hover_distance is None:
            if self.state_estimate:
                self.hover_distance = self.state_estimate.pose.position.z
            else:
                rospy.logwarn("No state estimate")
                return
        msg = Hover()
        msg.header.stamp = rospy.Time.now()
        msg.vx = velocity[0]
        msg.vy = velocity[1]
        msg.yawrate = -180 * angular_speed / np.pi
        self.hover_distance += velocity[2] * TAU
        msg.zDistance = self.hover_distance
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

    def publish_target_cmd(self, acc, vert_vel, angular_speed):
        # type: (np.ndarray, float, float) -> None
        rospy.error('publish_target_cmd not implemented')
        pass

    def hover(self, delta=[0, 0, 0]):
        # type: (np.ndarray) -> None
        if not self.state_estimate:
            rospy.logwarn("No state estimate")
            return
        with self.lock:
            if self.state == State.hovering:
                return
            # rospy.loginfo('hover')
            self.stop_hovering()
            p = self.state_estimate.pose.position
            q = self.state_estimate.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            target = np.array([p.x, p.y, p.z]) + delta

            # rospy.loginfo("Start hovering at %s %s", target, yaw)

            def callback(event):  # type: (rospy.Timer) -> None
                self.publish_target(target, yaw, hovering=True)
            self.publish_target(target, yaw, hovering=True)

            self.hover_timer = rospy.Timer(rospy.Duration(0.25), callback, oneshot=False)

    def update_state(self, msg):
        # type: (FlightState) -> None
        self.last_state_update = rospy.Time.now()
        if msg.battery_low:
            rospy.loginfo("update_state => battery critical")
            self.battery_state = BatteryState.critical
        else:
            self.battery_state = BatteryState.ok
        if self.state != State.landed and msg.thrust == 0:
            rospy.loginfo("update_state, thrust = 0 =>  landed")
            self.state = State.landed
        self.cf_can_fly = msg.can_fly

    def can_fly(self):
        # type: () -> bool
        return super(CFController, self).can_fly() and self.cf_can_fly

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
                    self.state = State.landed

        if self.state in [State.landing] and self.state_estimate is not None:
            z = self.state_estimate.pose.position.z
            if z < 0.1:
                self.stop()
        if self.state in [State.taking_off, State.flying]:
            if self.near_target():
                rospy.loginfo("update_state => is hovering")
                self.state = State.hovering

    def near_target(self):
        # type: () -> bool
        if self.hover_target_position is not None and self.state_estimate is not None:
            p = self.state_estimate.pose.position
            # rospy.loginfo("near target? %s %s", self.target_position, [p.x, p.y, p.z])
            dist = np.linalg.norm(np.array(self.hover_target_position) - np.array([p.x, p.y, p.z]))
            if dist > TOL:
                return False
            if self.hover_target_yaw is not None:
                q = self.state_estimate.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                return abs(np.unwrap([0, self.hover_target_yaw - yaw])[1]) < ANGLE_TOL
        return False

    def cf_land(self, msg):
        # type: (Empty) -> None
        self.land()

    def land(self):
        # type: (Empty) -> None
        rospy.loginfo("Land")
        p = self.state_estimate.pose.position
        self.state = State.landing
        self.hover(delta=[0, 0, -p.z])

    def takeoff(self):
        # type: (Empty) -> None
        rospy.loginfo("Takeoff")
        self.hover(delta=[0, 0, 0.5])
        self.state = State.taking_off

    def stop(self, msg=None):
        # type: (Optional[Empty]) -> None
        rospy.loginfo("Stop")
        self.stop_hovering()
        self.stop_pub.publish()
        self.state = State.landed

    def update_odometry(self, msg):
        # type: (Odometry) -> None
        self.state_estimate = msg.pose
