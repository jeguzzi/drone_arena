import math
from threading import RLock as Lock

import numpy as np

import rospy
from crazyflie_driver.msg import FlightState, Hover, Position
from crazyflie_driver.srv import UpdateParams
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

from .control import BatteryState, Controller, State, button

TAU = 0.02
ANGLE_TOL = 0.2
TOL = 0.1


class CFController(Controller):

    def give_feedback(self):
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
        with self.lock:
            if self.hover_timer:
                self.hover_timer.shutdown()
                self.hover_timer = None
                rospy.loginfo("Stop hovering")

    def publish_target_body_vel(self, velocity, angular_speed):
        # rospy.loginfo("Publish body vel %s %s", velocity, angular_speed)
        self.stop_hovering()
        super(CFController, self).publish_target_body_vel(velocity, angular_speed)
        if self.hover_distance is None:
            self.hover_distance = self.state_estimate.pose.position.z
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
        # DISABLED
        pass

    def hover(self, delta=[0, 0, 0]):
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

            def callback(event):
                self.publish_target(target, yaw, hovering=True)
            self.publish_target(target, yaw, hovering=True)

            self.hover_timer = rospy.Timer(rospy.Duration(0.25), callback, oneshot=False)

    def update_state(self, msg):
        if msg.battery_low:
            rospy.loginfo("update_state => battery critical")
            self.battery_state = BatteryState.critical

    def update(self, evt):
        super(CFController, self).update(evt)

        if self.state in [State.landing]:
            z = self.state_estimate.pose.position.z
            if z < 0.1:
                self.stop()
        if self.state in [State.taking_off, State.flying]:
            if self.near_target():
                rospy.loginfo("update_state => is hovering")
                self.state = State.hovering

    def near_target(self):

        if self.hover_target_position is not None:
            p = self.state_estimate.pose.position
            # rospy.loginfo("near target? %s %s", self.target_position, [p.x, p.y, p.z])
            if np.linalg.norm(np.array(self.hover_target_position) - np.array([p.x, p.y, p.z])) > TOL:
                return False
            if self.hover_target_yaw is not None:
                q = self.state_estimate.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                return abs(np.unwrap([0, self.hover_target_yaw - yaw])[1]) < ANGLE_TOL
        return False

    def __init__(self):
        super(CFController, self).__init__()

        self.lock = Lock()
        self.state_estimate = None
        self.hover_timer = None
        self.hover_distance = None
        self.hover_target_position = self.hover_target_yaw = None
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
        rospy.Subscriber('cf_odom', Empty, self.update_odometry, queue_size=1)

    def cf_land(self, msg):
        self.land()

    def land(self):
        rospy.loginfo("Land")
        p = self.state_estimate.pose.position
        self.state = State.landing
        self.hover(delta=[0, 0, -p.z])

    def takeoff(self):
        rospy.loginfo("Takeoff")
        self.hover(delta=[0, 0, 0.5])
        self.state = State.taking_off

    def stop(self, msg=None):
        rospy.loginfo("Stop")
        self.stop_hovering()
        self.stop_pub.publish()
        self.state = State.landed

    def update_odometry(self, msg):
        self.state_estimate = msg.pose
