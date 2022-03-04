#!/usr/bin/env python
# type: ignore
import rospy

from crazyflie_driver.srv import UpdateParams
from crazyflie_driver.msg import GenericLogData
from std_msgs.msg import Bool, Empty
import struct


class CFFrontNetMonitor(object):
    param = 'frontnet/enable_control'

    def set_state(self, value):
        cf_value = 1 if value else 0
        if cf_value == rospy.get_param(self.param, 0):
            return
        rospy.loginfo("Enable control -> %d", cf_value)
        rospy.set_param(self.param, cf_value)
        rospy.wait_for_service('update_params')
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        update_params([self.param])
        while True:
            rospy.sleep(1)
            if self.state != value:
                update_params([self.param])
            else:
                break

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        if value != self._state:
            rospy.loginfo(f'set frontnet state to {value}')
            self._state = value
            self.state_pub.publish(Bool(data=self._state))

    def __init__(self):
        rospy.init_node('frontnet_monitor')
        self.state_pub = rospy.Publisher('autonomous', Bool, queue_size=1, latch=True)
        self._state = True
        self.state = False
        rospy.Subscriber('frontnet_state', GenericLogData, self.got_state, queue_size=1)
        rospy.Subscriber('stop', Empty, self.got_stop, queue_size=1)
        rospy.Subscriber('frontnet/start', Empty, self.got_start, queue_size=1)
        rospy.spin()

    def got_state(self, msg):
        # HACK(Jerome): GenericLogData packs everything as float.
        # We try to pack and unpack to get the 1. byte value
        # (which apparently is 2 for true and 0 for false)
        # s = struct.pack('f', msg.values[0])
        # print('s', s, msg.values[0])
        self.state = msg.values[0] > 0

    def got_stop(self, msg):
        self.set_state(False)

    def got_start(self, msg):
        self.set_state(True)


if __name__ == '__main__':
    CFFrontNetMonitor()
