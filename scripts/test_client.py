#! /usr/bin/env python

import rospy
import actionlib
from drone_arena.msg import GoToPoseAction, GoToPoseGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
from itertools import cycle
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Empty

from drone_arena.temporized import Temporized

button = Temporized(1)


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
        self.client = actionlib.SimpleActionClient('fence_control', GoToPoseAction)
        self.home = pose(*rospy.get_param("~home", (0, 0, 1, 0)))
        self._poses = [pose(*x) for x in rospy.get_param("~plan", [])]
        self.loop = rospy.get_param("~loop", False)
        if self.loop:
            self.plan = cycle(self._poses)
        else:
            self.plan = iter(self._poses)
        self.done = False
        rospy.loginfo("home %s", self.home)
        rospy.loginfo("plan %s", self.plan)
        self.current_target = None
        rospy.loginfo('Waiting for fence_control action server')
        self.client.wait_for_server()
        rospy.loginfo('fence_control action server is there')
        self.running = False
        self.landing = False
        self.next_waypoint = None
        rospy.Subscriber("start", Empty, button(self.start_path))
        # rospy.Subscriber("stop", Empty, button(self.stop_path))
        rospy.Subscriber("land_home", Empty, button(self.land_home))
        self.land_pub = rospy.Publisher("land", Empty, queue_size=1)
        rospy.loginfo("fence client init done")
        while not rospy.is_shutdown():
            if self.running:
                if not self.next_waypoint:
                    try:
                        self.next_waypoint = self.plan.next()
                    except StopIteration:
                        self.running = False
                        self.done = True
                if self.next_waypoint:
                    goal = GoToPoseGoal(target_pose=self.next_waypoint)
                    self.move(goal)
            rospy.sleep(0.2)

    def move(self, goal):
        rospy.loginfo("Move to Goal %s", goal)
        self.client.send_goal(goal)
        finished_within_time = self.client.wait_for_result(
            rospy.Duration(60))
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.client.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            self.running = False
        else:
            # We made it!
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                self.next_waypoint = None
            elif state == GoalStatus.PREEMPTED:
                rospy.loginfo("Goal preempted!")
                self.running = False

    def land_home(self, msg):
        if self.landing:
            return
        self.landing = True
        rospy.loginfo("Go home and land")
        self.running = False
        self.client.cancel_all_goals()
        goal = GoToPoseGoal(target_pose=self.home)
        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(60)):
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("ready to land")
                rospy.sleep(1.0)
                self.land_pub.publish(Empty())
            else:
                rospy.loginfo("Failed moving to home %s",
                              self.client.get_state())
        self.landing = False

    def start_path(self, msg):
        if self.running:
            return
        rospy.loginfo("(re)Start a path")
        self.running = True
        if not self.loop and self.done:
            self.done = False
            self.next_waypoint = None
            self.plan = iter(self._poses)

    # def stop_path(self, msg):
    #     if self.running:
    #         if self.next_waypoint:
    #             self.client.cancel_goal()
    #         self.running = False


if __name__ == '__main__':
    try:
        Planner()
    except rospy.ROSInterruptException:
        print("Program interrupted")
