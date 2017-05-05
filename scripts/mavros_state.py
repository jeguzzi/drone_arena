#!/usr/bin/env python

import rospy
import bebop_msgs.msg as b_msg
from mavros_msgs.msg import State

from diagnostic import state_name


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
        rospy.spin()

    def has_received_state(self, msg):
        s = self.state[msg.state]
        mavros_msg = State()
        mavros_msg.header = msg.header
        mavros_msg.connected = True
        mavros_msg.armed = True
        mavros_msg.guided = s in ['flying']
        mavros_msg.mode = s
        if s in ['landed']:
            mavros_msg.system_status = 3  # MAV_STATE_STANDBY
        if s in ['hovering', 'flying', 'landing', 'usertakeoff', 'takingoff']:
            mavros_msg.system_status = 4  # MAV_STATE_ACTIVE
        elif s in ['emergency', 'emergency_landing']:
            mavros_msg.system_status = 6  # MAV_STATE_EMERGENCY
        self.mavros_pub.publish(mavros_msg)


if __name__ == '__main__':
    try:
        BebopMavRosState()
    except rospy.ROSInterruptException:
        pass
