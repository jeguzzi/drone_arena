#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger

if __name__ == '__main__':
    rospy.init_node('test')
    rospy.wait_for_service('safe_takeoff')
    rospy.wait_for_service('safe_land')
    takeoff = rospy.ServiceProxy('safe_takeoff', Trigger)
    land = rospy.ServiceProxy('safe_land', Trigger)
    while True:
        res = takeoff()
        if not res.success:
            break
        rospy.sleep(30)
        land()
        rospy.sleep(30)
