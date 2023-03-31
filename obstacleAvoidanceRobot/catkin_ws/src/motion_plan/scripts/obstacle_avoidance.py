#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

def callback_laser(msg):
  # 120 degrees into 3 regions
  regions = {
    'right':  min(min(msg.ranges[-60:-1]),2),
    'front':  min(min(msg.ranges[0:1]),2),
    'left':   min(min(msg.ranges[2:60]),2)
  }
  
  take_action(regions)
  
def take_action(regions):
  threshold_dist = 1.0 # 기준값 (임계값)
  linear_speed = 0.5 # 직선 스피드
  angular_speed = 0.5 # 회전 스피드

  msg = Twist()
  linear_x = 0 # 직선 각도
  angular_z = 0 # 회전 각도
  
  state_description = ''
  
  if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 1 - no obstacle' # 장애물이 없을 때
    linear_x = linear_speed
    angular_z = 0
  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 7 - front and left and right' # 장애물이 앞, 왼, 오른쪽에 있을 때
    linear_x = 0
    angular_z = -angular_speed # Increase this angular speed for avoiding obstacle faster
  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 2 - front' # 정면에 장애물이 있을때
    linear_x = 0
    angular_z = -angular_speed
  elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 3 - right' # 오른쪽에 장애물이 있을때
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 4 - left' # 왼쪽에 장애물이 있을때
    linear_x = 0
    angular_z = -angular_speed
  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 5 - front and right' # 정면에서 오른쪽 대각에 장애물이 있을때
    linear_x = 0
    angular_z = angular_speed
  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
    state_description = 'case 6 - front and left' # 정면에서 왼쪽 대각에 장애물이 있을때
    linear_x = 0
    angular_z = -angular_speed
  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
    state_description = 'case 8 - left and right' # 왼쪽과 오른쪽에 장애물이 있을때 직진
    linear_x = linear_speed
    angular_z = 0
  else:
    state_description = 'unknown case' # 아무것도 없었을 때
    rospy.loginfo(regions)

  rospy.loginfo(state_description)
  msg.linear.x = linear_x
  msg.angular.z = angular_z
  pub.publish(msg)

def main():
  global pub
  
  rospy.init_node('reading_laser')
  
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  
  sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
  
  rospy.spin()

if __name__ == '__main__':
  main()
