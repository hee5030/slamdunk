#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


waypoints = [
    ['one', (1.1, 1.2), (0.0, 0.0, 0.0, 1.0)],
    ['two', (2.1, 5.0), (0.0, 0.0, -0.984047240305, 0.177907360295)],
    ['three', (-1.3, 5.4), (0.0, 0.0, 0.0, 1.0)],
    ['four', (-1.3, 1.), (0.0, 0.0, 0.0, 1.0)],
    ['five', (2.1, 2.2), (0.0, 0.0, 0.0, 1.0)]
]

num_points = 5

def goal_pose(pose):  # <2>
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('patrol')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()
    
    while True:
        for pose in waypoints:   # <4>
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()

        # goal_pose = MoveBaseGoal()
        # goal_pose.target_pose.header.frame_id = 'map'
        # goal_pose.target_pose.pose.position.x = pose[0]
        # goal_pose.target_pose.pose.position.y = pose[1]
        # goal_pose.target_pose.pose.position.z = 0.0
        # goal_pose.target_pose.pose.orientation.x = pose[0]
        # goal_pose.target_pose.pose.orientation.y = pose[1]
        # goal_pose.target_pose.pose.orientation.z = pose[2]
        # goal_pose.target_pose.pose.orientation.w = pose[3]