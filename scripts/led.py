#!/usr/bin/env python

from __future__ import division

import rospy
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from lawa.blinkstick_driver import blinkstickROS
from lawa.battery import line, single


class DroneLed(blinkstickROS):
    def __init__(self):
        super(DroneLed, self).__init__()

        self.battery_percent = None
        self.location = None
        self.last_ping = None
        rospy.Subscriber('battery', BatteryState, self.has_received_battery)
        rospy.Subscriber('location', String, self.has_received_location)
        rospy.Subscriber('odom', Odometry, self.has_received_odometry)

        self.type = rospy.get_param('~type', 'line')
        if self.type == 'line':
            min_percent = 100 / self.number_of_leds
        else:
            min_percent = 10
        self.min_percent = rospy.get_param('~min_percent', min_percent)
        self.max_period = rospy.get_param('~max_period', 0.5)
        self.min_period = rospy.get_param('~min_period', 0.1)
        self.period = rospy.get_param('~period', 1.0)

        rospy.Timer(rospy.Duration(self.period), self.update)

    def has_received_battery(self, msg):
        self.battery_percent = 100 * msg.percentage

    def has_received_location(self, msg):
        self.location = msg.data

    def has_received_odometry(self, msg):
        self.last_ping = rospy.Time.now()

    def alive(self):
        return self.last_ping and (rospy.Time.now() - self.last_ping).to_sec() < 1.0

    def update(self, evt):
        all = list(range(self.number_of_leds))
        delay = self.period * 500
        if not self.alive():
            self.blink(delay=delay, name='blue', index=all)
            return
        if self.location in ['', None]:
            self.blink(delay=delay, name='purple', index=all)
            return
        if self.location == 'out':
            self.blink(delay=delay, name='yellow', index=all)
            return
        if self.battery_percent is None:
            self.set_color(index=all, name='cyan')
            return
        if self.battery_percent < self.min_percent:
            duration = (
                self.max_period +
                ((self.min_percent - self.max_percent) * self.battery_percent / self.min_percent))
            repeats = int(self.period / duration)
            self.blink(name='red', index=0, repeats=repeats, delay=duration * 1000 / 2)
        if self.type == 'line':
            color = line(self.battery_percent, self.number_of_leds)
        else:
            color = single(self.battery_percent)
        # rospy.loginfo(color)
        self.set_color(index=color)


if __name__ == '__main__':
    DroneLed()
    rospy.spin()
