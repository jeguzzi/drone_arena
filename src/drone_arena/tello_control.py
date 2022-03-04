import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from typing import Tuple  # noqa
import numpy as np
from .control import BatteryState, State, Controller
from tello_driver.msg import TelloStatus
from sensor_msgs.msg import BatteryState as BatteryStateMsg

# From the manual, page 5:
# The Tello has two flight speeds that you can select when flying the aircraft manually:
# - Slow (default): The maximum flight attitude angle is 9 degrees and the maximum flight speed is 8.9 mph (14.4 kph).
# - Fast: The maximum flight attitude angle is 25 degrees and the maximum flight speed is 17.8 mph (28.8 kph).

# F = 2.83
F = 1.54  # 9 degrees -> tan(max_pitch) * g ~= 1.54
F_fast = 4.28
# F = F_fast
W = 1.55  # DONE: calibrated


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


class TelloController(Controller):

    # ---- concrete implementation

    @staticmethod
    def cmd_from(acc, speed_z, omega):
        # type: (np.ndarray, float, float) -> Twist
        msg = Twist()
        msg.linear = Vector3(clamp(acc[0] / F, -1, 1), clamp(acc[1] / F, -1, 1), speed_z)
        msg.angular = Vector3(0, 0, clamp(omega / W, -1, 1))
        return msg

    @staticmethod
    def from_cmd(pitch, roll, z, omega):
        # type: (float, float, float, float) -> Tuple[Tuple[float, float, float], float, float]
        return ((F * pitch, F * roll, 0), z, omega * W)

    # TODO: parametrize
    def give_feedback(self):
        # type: () -> None
        A = rospy.get_param('~feedback/amplitude', 1)
        n = rospy.get_param('~feedback/movements', 1)
        t_up = rospy.get_param('~feedback/up', 0.15)
        t_down = rospy.get_param('~feedback/down', 0.5)
        rospy.loginfo("Start gesture with A %s, n %s, dt (%s, %s)", A, n, t_up, t_down)
        for _ in range(n):
            for dz, dt in zip([A, -A], [t_up, t_down]):
                self.des_cmd_pub.publish(Twist(linear=Vector3(0, 0, dz / dt)))
                rospy.sleep(dt)
        self.des_cmd_pub.publish(Twist(linear=Vector3(0, 0, 0)))

    def stop(self, msg=None):
        # type: (Empty) -> None
        self.pub_reset.publish()

    def hover(self):
        # type: () -> None
        if self.state == State.flying:
            self.des_cmd_pub.publish(Twist())

    def takeoff(self):
        # type: () -> None
        self.pub_takeoff.publish(Empty())

    def land(self):
        # type: () -> None
        self.pub_land.publish(Empty())

    def __init__(self):
        # type: () -> None
        self.tello_is_safe = False
        super(TelloController, self).__init__()
        self.battery_pub = rospy.Publisher('battery', BatteryStateMsg, queue_size=1, latch=True)
        self.battery_msg = BatteryStateMsg()
        self.battery_msg.header.frame_id = 'battery'
        self.battery_msg.voltage = 3.8
        self.battery_msg.current = self.battery_msg.charge = self.battery_msg.capacity = float('nan')
        self.battery_msg.design_capacity = 1.1
        self.power_supply_status = BatteryStateMsg.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_msg.present = True
        self.battery_state = BatteryState.ok
        rospy.Subscriber('status', TelloStatus, self.state_has_changed)
        self.pub_takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('land', Empty, queue_size=1)
        self.pub_reset = rospy.Publisher('reset', Empty, queue_size=1)

    def can_fly(self):
        # type: () -> bool
        if not self.tello_is_safe:
            rospy.logwarn('Tello is not safe')
            return False
        return super(TelloController, self).can_fly()

    def state_has_changed(self, msg):
        # type: (TelloStatus) -> None
        # TODO: complete

        # TODO: check
        # flight_data.equipment = data.electrical_machinery_state
        # flight_data.high_temperature = data.temperature_height

        self.battery_percent = msg.battery_percentage
        self.battery_msg.percentage = msg.battery_percentage / 100.0
        self.battery_pub.publish(self.battery_msg)
        if msg.is_battery_lower:
            self.battery_state = BatteryState.critical
        elif msg.is_battery_low:
            self.battery_state = BatteryState.empty
        else:
            self.battery_state = BatteryState.ok
        # 12 landing
        # 6 landed ma anche hovering
        # 11 takeoff ? or flying
        # 1 ?
        if msg.fly_mode == 6:
            if not msg.is_flying:
                self.state = State.landed
            else:
                if any([msg.cmd_roll_ratio, msg.cmd_pitch_ratio,
                        msg.cmd_yaw_ratio, msg.cmd_vspeed_ratio]):
                    self.state = State.flying
                else:
                    self.state = State.hovering
        elif msg.fly_mode == 11:
            self.state = State.taking_off
        elif msg.fly_mode == 12:
            self.state = State.landing
        elif msg.fly_mode == 1:
            # When motors are off but drone is beeing moved around
            self.state = State.landed

        self.tello_is_safe = msg.electrical_machinery_state != 21
        self.tello_is_overheating = msg.electrical_machinery_state == 251
