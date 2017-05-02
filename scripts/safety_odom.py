#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from drone_arena.cfg import SafeOdomConfig
import diagnostic_updater
import diagnostic_msgs


class SafetyOdom(object):
    """docstring for IMU."""
    def __init__(self):
        rospy.init_node('safety_odom')
        self.last_msg = []
        self.pub = rospy.Publisher('out', Odometry, queue_size=1)
        self.active = rospy.get_param('~active', True)
        self.timeout = rospy.get_param('~timeout', 1.0)
        self.active_index = 0
        self.srv = Server(SafeOdomConfig, self.reconfigure)
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("odom sources")
        self.topics = []
        for i, data in enumerate(rospy.get_param('~in', [])):
            topic = data['topic']
            source = data.get('source', topic)
            self.last_msg.append(rospy.Time.now())
            self.topics.append(topic)
            rospy.Subscriber(source, rospy.AnyMsg, self.got_source(i))
            rospy.Subscriber(topic, Odometry, self.got_odometry_from(i))
            self.updater.add(topic, self.diagnostics(i))
        self.updater.add('active source', self.active_diagnostics)
        rospy.Timer(rospy.Duration(1), self.update_diagnostics)
        rospy.spin()

    def got_source(self, i):
        def f(msg):
            self.last_msg[i] = rospy.Time.now()
        return f

    def update_diagnostics(self, event):
        self.updater.update()

    def active_diagnostics(self, stat):
        index = self.active_index
        topic = self.topics[index]
        dt = (self.last_msg[index] - rospy.Time.now()).to_sec()
        if dt > self.timeout:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                         "Active source {0} not alive".format(topic))
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                         "Active source {0} alive".format(topic))
        return stat

    def diagnostics(self, index):
        OK = diagnostic_msgs.msg.DiagnosticStatus.OK
        WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN

        def f(stat):
            dt = (self.last_msg[index] - rospy.Time.now()).to_sec()
            if dt > self.timeout:
                stat.summary(WARN, "Last updated {0:.0f} seconds ago".format(dt))
            else:
                stat.summary(OK, "")
            return stat
        return f

    def got_odometry_from(self, index):
        def f(msg):
            if self.active:
                if index <= self.active_index:
                    self.active_index = index
                else:
                    dt = (self.last_msg[self.active_index] - rospy.Time.now()).to_sec()
                    if dt > self.timeout:
                        rospy.logwarn('Switch to odom source {0} from {1}'.format(
                            self.topics[index], self.topics[self.active_index]))
                        self.active_index = index
            if index == self.active_index:
                self.pub.publish(self.imu_msg)
        return f

    def reconfigure(self, config, level):
        self.active = config['active']
        if not self.active:
            config['active_index'] = min(config['active_index'], len(self.topics) - 1)
            rospy.logwarn('Switch to odom source {0} from {1}'.format(
                self.topics[config['active_index']], self.topics[self.active_index]))
            self.active_index = config['active_index']
        return config


if __name__ == '__main__':
    SafetyOdom()
