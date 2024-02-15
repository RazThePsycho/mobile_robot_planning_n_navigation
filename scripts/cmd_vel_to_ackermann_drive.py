#!/usr/bin/env python3

# Author: christoph.roesmann@tu-dortmund.de
# Modified: Vvlad1slavV@yandex.ru

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

MAX_STEERING_ANGLE = math.pi/4

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  steering_angle = math.atan(wheelbase / radius)
  if steering_angle>MAX_STEERING_ANGLE:
    steering_angle = MAX_STEERING_ANGLE
  if steering_angle<-MAX_STEERING_ANGLE:
    steering_angle = -MAX_STEERING_ANGLE
  return steering_angle


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  msg.drive.steering_angle_velocity = 0.01
  msg.drive.speed = v
  # msg.drive.acceleration = 1
  # msg.drive.jerk = 0.1
  
  pub.publish(msg)
  


if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/robot/move_base/cmd_vel')
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/robot/robot_cmd_vel')
    wheelbase = rospy.get_param('~wheelbase', 1.83)
    frame_id = rospy.get_param('~frame_id', 'robot_base_link')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", twist_cmd_topic, ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

