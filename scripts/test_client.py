#! /usr/bin/env python

import rospy
import actionlib
from drone_arena.msg import GoToPoseAction, GoToPoseGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from itertools import cycle
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Empty


def pose(x, y, z, yaw):
    q = quaternion_from_euler(0, 0, yaw)
    p = [x, y, z]
    r = PoseStamped()
    r.header.frame_id = 'World'
    r.pose.position = Point(*p)
    r.pose.orientation = Quaternion(*q)
    return r


class Planner():
    def __init__(self):
        rospy.init_node('fence_client')
        self.client = actionlib.SimpleActionClient(
            'fence_control', GoToPoseAction)
        self.home = pose(*rospy.get_param("~home", (0, 0, 1, 0)))
        poses = [pose(*x) for x in rospy.get_param("~plan", [])]
        loop = rospy.get_param("~loop", False)
        if loop:
            self.plan = cycle(poses)
        else:
            self.plan = iter(poses)
        rospy.loginfo("home %s", self.home)
        rospy.loginfo("plan %s", self.plan)
        self.current_target = None
        self.client.wait_for_server()
        self.running = False
        self.next_waypoint = None
        rospy.Subscriber("start", Empty, self.start_path)
        rospy.Subscriber("stop", Empty, self.stop_path)
        rospy.Subscriber("land_home", Empty, self.land_home)
        rospy.loginfo("init done")
        while not rospy.is_shutdown():
            if self.running:
                if not self.next_waypoint:
                    try:
                        self.next_waypoint = self.plan.next()
                    except StopIteration:
                        self.running = False
                if self.next_waypoint:
                    goal = GoToPoseGoal(target_pose=self.next_waypoint)
                    self.move(goal)
            rospy.sleep(1)

    def move(self, goal):
        rospy.loginfo("Move to Goal %s", goal)
        self.client.send_goal(goal)
        finished_within_time = self.client.wait_for_result(
            rospy.Duration(60))
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                self.next_waypoint = None
            elif state == GoalStatus.PREEMPTED:
                rospy.loginfo("Goal preempted!")

    def land_home(self, msg):
        rospy.loginfo("Go home and land")
        goal = GoToPoseGoal(target_pose=self.home)
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(60))
        rospy.loginfo("landed: %s" % self.client.get_state())

    def start_path(self, msg):
        rospy.loginfo("(re)Start a path")
        self.running = True

    def stop_path(self, msg):
        if self.running:
            if self.next_waypoint:
                self.client.cancel_goal()
            self.running = False


if __name__ == '__main__':
    try:
        Planner()
    except rospy.ROSInterruptException:
        print "Program interrupted"
