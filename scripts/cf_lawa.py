#!/usr/bin/env python

from lawa import lawa
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from crazyflie_driver.msg import FlightState
import rospy
INDICES = list(range(8))
BATTERY_COLOR_PERIOD = 0.95


@lawa.add('battery', BatteryState)
def battery(msg, bs):
    rospy.loginfo("Battery")
    if bs.battery_color is None:
        return
    charging = (msg.power_supply_status in [BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                                            BatteryState.POWER_SUPPLY_STATUS_FULL])
    duration = msg.percentage * 0.01 * BATTERY_COLOR_PERIOD
    if charging:
        bs.blink(name=bs.battery_color, index=INDICES, repeats=1, delay=duration * 1000)
        return
    if (msg.percentage < 10 or bs.battery_low) and not charging:
        bs.blink(name='red', index=INDICES, repeats=5,
                 delay=BATTERY_COLOR_PERIOD * 100)
        return
    rospy.loginfo("duration %s", duration)
    bs.blink(name=bs.battery_color, index=INDICES, repeats=1, delay=duration * 1000)
    rospy.loginfo("Done")


@lawa.add('connected', Bool)
def connected(msg, bs):
    if not msg.data:
        bs.set_color(name='magenta', index=INDICES)
        bs.battery_color = None


@lawa.add('state', FlightState)
def state(msg, bs):
    if msg.flying and msg.thrust > 0:
        bs.battery_color = 'blue'
    elif msg.can_fly:
        bs.battery_color = 'green'
    else:
        bs.battery_color = 'yellow'
    bs.battery_low = msg.battery_low


def init(bs):
    bs.battery_color = None


if __name__ == '__main__':
    lawa.run(init=init)
