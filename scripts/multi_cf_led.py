#!/usr/bin/env python

from __future__ import division

import rospy
from crazyflie_driver.msg import FlightState
from lawa.blinkstick_driver import blinkstickROS
from sensor_msgs.msg import BatteryState


class CFLed(blinkstickROS):
    def __init__(self):
        super(CFLed, self).__init__()
        self.battery = {}
        self.state = {}
        self.last_ping = {}
        self.min_percent = rospy.get_param('~min_percent', 10.0)
        self.period = rospy.get_param('~period', 1.0)
        self.led_map = rospy.get_param('~led_map', {})
        for name in self.led_map:
            rospy.Subscriber('{0}/state'.format(name), FlightState, self.has_received_state, name)
            rospy.Subscriber('{0}/battery'.format(name), BatteryState, self.has_received_battery,
                             name)
        rospy.Timer(rospy.Duration(self.period), self.update)
        rospy.on_shutdown(self.shutdown)

    def has_received_battery(self, msg, name):
        self.battery[name] = msg
        self.last_ping[name] = rospy.Time.now()

    def has_received_state(self, msg, name):
        self.state[name] = msg
        self.last_ping[name] = rospy.Time.now()

    def alive(self, name):
        return name in self.last_ping and (rospy.Time.now() - self.last_ping[name]).to_sec() < 2.0

    def update(self, evt):
        for name, all in self.led_map.items():
            if not self.alive(name):
                self.set_color(name='magenta', index=all)
                continue

            if name not in self.battery:
                self.set_color(name='red', index=all)
                continue

            if name not in self.state:
                self.set_color(name='orange', index=all)
                continue

            if self.state[name].flying and self.state[name].thrust > 0:
                color = 'blue'
            elif self.state[name].can_fly:
                color = 'green'
            else:
                color = 'yellow'
            if self.battery[name].percentage < 0.2:
                color = 'yellow'
            if self.battery[name].percentage < 0.15:
                color = 'orange'

            charging = (self.battery[name].power_supply_status in [
                BatteryState.POWER_SUPPLY_STATUS_CHARGING, BatteryState.POWER_SUPPLY_STATUS_FULL])
            duration = self.battery[name].percentage * self.period * 1000
            if charging:
                self.blink(name='cyan', index=all, repeats=1, delay=duration)
                continue
            if((self.battery[name].percentage < 0.1 or self.state[name].battery_low) and
               not charging):
                self.blink(name='red', index=all, repeats=5, delay=self.period * 100)
                continue
            self.blink(name=color, index=all, repeats=1, delay=duration)

    def shutdown(self):
        all = list(range(self.number_of_leds))
        self.set_color(name='black', index=all)


if __name__ == '__main__':
    cfled = CFLed()
    rospy.spin()
