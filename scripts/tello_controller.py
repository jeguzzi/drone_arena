#!/usr/bin/env python

import rospy

from drone_arena.tello_control import TelloController

if __name__ == '__main__':
    try:
        TelloController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
