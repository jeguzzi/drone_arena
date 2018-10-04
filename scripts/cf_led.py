#!/usr/bin/env python

from __future__ import division

import rospy
from crazyflie_driver.msg import FlightState
from lawa.blinkstick_driver import blinkstickROS
from sensor_msgs.msg import BatteryState


class CFLed(blinkstickROS):
    def __init__(self):
        super(CFLed, self).__init__()

        self.battery = None
        self.state = None
        self.last_ping = None
        self.min_percent = rospy.get_param('~min_percent', 10.0)
        self.period = rospy.get_param('~period', 1.0)

        rospy.Subscriber('state', FlightState, self.has_received_state)
        rospy.Subscriber('battery', BatteryState, self.has_received_battery)
        rospy.Timer(rospy.Duration(self.period), self.update)
        rospy.on_shutdown(self.shutdown)

    def has_received_battery(self, msg):
        self.battery = msg
        self.last_ping = rospy.Time.now()

    def has_received_state(self, msg):
        self.state = msg
        self.last_ping = rospy.Time.now()

    @property
    def alive(self):
        return self.last_ping and (rospy.Time.now() - self.last_ping).to_sec() < 2.0

    def update(self, evt):
        all = list(range(self.number_of_leds))
        if not self.alive:
            self.set_color(name='magenta', index=all)
            return

        if not self.battery:
            self.set_color(name='red', index=all)
            return

        if not self.state:
            self.set_color(name='orange', index=all)
            return

        if self.state.flying and self.state.thrust > 0:
            color = 'blue'
        elif self.state.can_fly:
            color = 'green'
        else:
            color = 'yellow'
        if self.battery.percentage < 0.2:
            color = 'yellow'
        if self.battery.percentage < 0.15:
            color = 'orange'


        charging = (self.battery.power_supply_status in [BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                                                         BatteryState.POWER_SUPPLY_STATUS_FULL])
        duration = self.battery.percentage * self.period * 1000
        if charging:
            self.blink(name='cyan', index=all, repeats=1, delay=duration)
            return
        if (self.battery.percentage < 0.1 or self.state.battery_low) and not charging:
            self.blink(name='red', index=all, repeats=5, delay=self.period * 100)
            return
        self.blink(name=color, index=all, repeats=1, delay=duration)

    def shutdown(self):
        all = list(range(self.number_of_leds))
        self.set_color(name='orange', index=all)

if __name__ == '__main__':
    cfled = CFLed()
    rospy.spin()
