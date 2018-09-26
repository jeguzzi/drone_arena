#!/usr/bin/env python

import rospy

from drone_arena.bebop_control import BebopController

if __name__ == '__main__':
    try:
        BebopController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
