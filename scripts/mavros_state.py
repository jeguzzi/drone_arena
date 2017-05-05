#!/usr/bin/env python

import rospy
import bebop_msgs.msg as b_msg
from mavros_msgs.msg import State

from diagnostics import state_name


class BebopMavRosState(object):

    def __init__(self):
        rospy.init_node('mavros_state_pub', anonymous=True)
        rospy.set_param('bebop_driver/states/enable_pilotingstate_flyingstatechanged', True)
        self.state = {state_name(name): name
                      for name in ['landed', 'hovering', 'flying', 'landing', 'usertakeoff',
                                   'takingoff', 'emergency', 'emergency_landing']}
        self.mavros_pub = rospy.Publisher('mavros/state', State, queue_size=1)
        rospy.Subscriber('states/ardrone3/PilotingState/FlyingStateChanged',
                         b_msg.Ardrone3PilotingStateFlyingStateChanged,
                         self.has_received_state)
        self.mavros_msg = State()
        self.mavros_msg.connected = True
        self.mavros_msg.armed = True
        rospy.Timer(rospy.Duration(1), self.publish_mavros_state)
        rospy.spin()

    def publish_mavros_state(self, evt):
        self.mavros_pub.publish(self.mavros_msg)

    def has_received_state(self, msg):
        s = self.state[msg.state]
        self.mavros_msg.header = msg.header
        self.mavros_msg.guided = s in ['flying']
        self.mavros_msg.mode = s
        if s in ['landed']:
            self.mavros_msg.system_status = 3  # MAV_STATE_STANDBY
        if s in ['hovering', 'flying', 'landing', 'usertakeoff', 'takingoff']:
            self.mavros_msg.system_status = 4  # MAV_STATE_ACTIVE
        elif s in ['emergency', 'emergency_landing']:
            self.mavros_msg.system_status = 6  # MAV_STATE_EMERGENCY


if __name__ == '__main__':
    try:
        BebopMavRosState()
    except rospy.ROSInterruptException:
        pass
