#! /usr/bin/env python

import rospy
import actionlib
from hl_navigation_msgs.msg import GoToTargetAction, GoToTargetGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Point, PointStamped
from tf.transformations import quaternion_from_euler
from itertools import cycle
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Empty

from drone_arena.temporized import Temporized
from drone_arena_msgs.msg import TargetSource

button = Temporized(1)


def pose(x, y, z, yaw):
    q = quaternion_from_euler(0, 0, yaw)
    p = [x, y, z]
    r = PoseStamped()
    r.header.frame_id = 'World'
    r.pose.position = Point(*p)
    r.pose.orientation = Quaternion(*q)
    return r


def point(x, y, z, *args):
    p = [x, y, z]
    r = PointStamped()
    r.header.frame_id = 'World'
    r.point = Point(*p)
    return r


class Planner():

    def __init__(self):
        rospy.init_node('nav_client')
        self.client = actionlib.SimpleActionClient(
            'go_to_target', GoToTargetAction)
        self._poses = [point(*x) for x in rospy.get_param("~nav_plan", [])]
        # self.loop = rospy.get_param("~loop", False)
        self.loop = True
        if self.loop:
            self.plan = cycle(self._poses)
        else:
            self.plan = iter(self._poses)
        self.done = False
        rospy.loginfo("plan %s", self.plan)
        self.current_target = None
        self.client.wait_for_server()
        self.running = False
        self.landing = False
        self.next_waypoint = None
        self.last_input = None
        self.source_pub = rospy.Publisher("target_source", TargetSource, queue_size=1)
        self.source_msg = TargetSource(mode=TargetSource.BodyVel, topic='target/body_vel')
        rospy.Subscriber("start_nav", Empty, button(self.start_path))
        rospy.loginfo("nav_client init done")
        while not rospy.is_shutdown():
            if self.running:
                if not self.next_waypoint:
                    try:
                        self.next_waypoint = self.plan.next()
                    except StopIteration:
                        self.running = False
                        self.done = True
                if self.next_waypoint:
                    goal = GoToTargetGoal(target_point=self.next_waypoint,
                                          target_pose=PoseStamped())
                    self.move(goal)
            rospy.sleep(0.2)

    def move(self, goal):
        rospy.loginfo("Move to Goal %s", goal)
        self.client.send_goal(goal)
        finished_within_time = self.client.wait_for_result(rospy.Duration(60))
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
            elif state == GoalStatus.ABORTED:
                rospy.loginfo("Goal aborted!")
                self.running = False

    def start_path(self, msg):
        if self.last_input and (rospy.Time.now() - self.last_input).to_sec() < 1:
            rospy.loginfo('Ignore start command')
            return
        self.last_input = rospy.Time.now()
        if self.running:
            rospy.loginfo('Already running')
            return
        rospy.loginfo("(re)Start a path")
        self.running = True
        self.source_pub.publish(self.source_msg)
        if not self.loop and self.done:
            self.done = False
            self.next_waypoint = None
            self.plan = iter(self._poses)


if __name__ == '__main__':
    try:
        Planner()
    except rospy.ROSInterruptException:
        print("Program interrupted")
