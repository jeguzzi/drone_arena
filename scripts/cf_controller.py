#!/usr/bin/env python

import rospy

from drone_arena.cf_control import CFController

if __name__ == '__main__':
    try:
        CFController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
