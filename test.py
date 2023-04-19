# -*- coding: utf-8 -*- 
#!/usr/bin/env python

import rospy
from smach import State,StateMachine
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State,StateMachine
import subprocess

from time import sleep
import sys
import select
import tty
import termios
import smach_ros



def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []) # 문자열을 읽을 수 있는 함수



# waypoints = [
#         ['one', (1.1, 0.5), (0.0, 0.0, 0.0, 1.0)],
#         ['two', (2.1, 4.43), (0.0, 0.0, -0.984047240305, 0.177907360295)],
#         ['three', (-1.3, 4.4), (0.0, 0.0, 0.0, 1.0)],
#         ['four', (-1.3, 0.1), (0.0, 0.0, 0.0, 1.0)]
#     ] 

waypoints = [
        ['one', (2.64499974251, 0.509999752045), (0.0, 0.0, 0.707106796641, 0.707106765732)],
        ['two', (2.47258925438, 4.40223407745), (0.0, 0.0, 0.999878011069, -0.0156193143396)],
        ['three', (-1.40137755871, 4.37288618088), (0.0, 0.0, -0.703452686599, 0.71074208945)],
        ['four', (-1.32800757885, 0.0733705535531), (0.0, 0.0, 0.0142813322057, 0.999898016575)]
    ]

roslaunch_process = None
class one(State): # one의 지점으로 가는 상태를 나타내는 구문
    def __init__(self, position, orientation): 
        State.__init__(self, outcomes=['success', 'stop1'])

         # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server() # action client을 얻는 것 movebaseaction 

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3] # movebasegoal message에서 나오는 형식

    def execute(self, userdata): # 실행하는 함수 execute
        print('one')
        self.client.send_goal(self.goal)
        
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 's':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop1'
        self.client.wait_for_result() # goal을 보내고나서 s키를 누르면 골을 취소하고 stop으로 보낸뒤 기다린다. 
        print('success')
        return 'success'

class two(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'stop2'])

         # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self, userdata):
        print('two')
        self.client.send_goal(self.goal)
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 's':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop2'
        print(self.client.get_result())
        self.client.wait_for_result()
        print('success')
        return 'success'

class three(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'stop3'])

         # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self, userdata):
        print('three')
        self.client.send_goal(self.goal)
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 's':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop3'
        print(self.client.get_result())
        self.client.wait_for_result()
        print('success')
        return 'success'

class four(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success', 'stop4'])

         # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self, userdata):
        print('four')
        self.client.send_goal(self.goal)
        while(not self.client.get_result()):
            if isData():
                c = sys.stdin.read(1)
                if c == 's':
                    self.client.cancel_all_goals()
                    print('stop')
                    return 'stop4'
        print(self.client.get_result())
        self.client.wait_for_result()
        print('success')
        return 'success'

class stop1(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    
    def execute(self, userdata):  
        self.client.cancel_all_goals()
        while True:
            if isData():
                c = sys.stdin.read(1)
                if c == 'k':
                    break # 'k'를 누르면 break 실행
        print('success')
        return 'success' # success로 보낸다

class stop2(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    
    def execute(self, userdata):  
        self.client.cancel_all_goals()
        while True:
            if isData():
                c = sys.stdin.read(1)
                if c == 'k':
                    break # 'k'를 누르면 break 실행
        print('success')
        return 'success' # success로 보낸다
    
class stop3(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    
    def execute(self, userdata):
        self.client.cancel_all_goals()
        while True:
            if isData():
                c = sys.stdin.read(1)
                if c == 'k':
                    break # 'k'를 누르면 break 실행
        print('success')
        return 'success' # success로 보낸다
    
class stop4(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    
    def execute(self, userdata):
        self.client.cancel_all_goals()
        while True:
            if isData():
                c = sys.stdin.read(1)
                if c == 'k':
                    break # 'k'를 누르면 break 실행
        print('success')
        return 'success' # success로 보낸다




if __name__ == '__main__':
    rospy.init_node('sm') # 노드 형성

    sm = StateMachine(outcomes=['success']) # StateMchine 상태 변환
    with sm:
        StateMachine.add('one', one(waypoints[0][1], waypoints[0][2]), transitions={'success':'two', 'stop1':'stop1'}) # success를 하면 하나의 지점으로 이동하고 stop을 하면 정지하는 변환 수행
        StateMachine.add('two', two(waypoints[1][1], waypoints[1][2]), transitions={'success':'three', 'stop2':'stop2'})
        StateMachine.add('three', three(waypoints[2][1], waypoints[2][2]), transitions={'success':'four', 'stop3':'stop3'})
        StateMachine.add('four', four(waypoints[3][1], waypoints[3][2]), transitions={'success':'one', 'stop4':'stop4'})
        StateMachine.add('stop1', stop1(), transitions={'success': 'one'})
        StateMachine.add('stop2', stop2(), transitions={'success': 'two'})
        StateMachine.add('stop3', stop3(), transitions={'success': 'three'})
        StateMachine.add('stop4', stop4(), transitions={'success': 'four'})
        
    # sm.execute()

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT') # smach_viewer를 보기 위한 구문 IntrospectionServer를 먼저 작동
    sis.start()
    outcome = sm.execute() # 실행
    rospy.spin() # spin으로 여기까지 계속 실행
    sis.stop() # 정지1
