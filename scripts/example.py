#!/usr/bin/env python3

import time
from math import sin
import numpy
import itertools

import rospy


from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimpleMover():

    def __init__(self):
        rospy.init_node('example_node', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel_pub = rospy.Publisher('/robot/robot_cmd_vel', AckermannDriveStamped, queue_size=1)
        rospy.Subscriber("/robot/robot_front_ptz_camera/image_raw", Image, self.camera_cb)
        rospy.Subscriber("/robot/teraranger_duo/sonar", Range, self.sonar)
        rospy.Subscriber("/robot/lidar_3d/points", PointCloud2, self.lidar)
        rospy.Subscriber("/robot/odometry/gps", Odometry, self.gps)
        self.rate = rospy.Rate(30)
        self.cv_bridge = CvBridge()
        global gui
        gui = rospy.get_param("/gui", False)

    def camera_cb(self, msg):

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        if gui == True:
            self.show_image(cv_image)
    
    def sonar(self, msg):
        pass
        # print ("Sonar range is: ", msg.range, "\n")

    def lidar(self, msg):
        pass
        # print ("Advertisment to lidar topic. Is dense?", msg.is_dense, "\n")

    def gps(self, msg):
        pass
        # print ("The position of baggy is:", msg.pose.pose.position.x, " ", msg.pose.pose.position.y, " ", msg.pose.pose.position.z, "\n" )


    def show_image(self, img):
        cv2.imshow("Front Camera", img)
        cv2.waitKey(3)

    def move(self):

        steering_angle = []
        steering_angle_zero = []
        v = []
        v1 = []
        for i in range(560):
            steering_angle.append(3.14/8)

        for i in range (560):
            v1.append(3.0)
        
        for i in range(70):
            steering_angle_zero.append(0.0)

        for i in range (70):
            v.append(3.0)
        
        msg = AckermannDriveStamped()
        for i in range(550) :
                print("I'm in cycle")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "robot_odom"
                msg.drive.steering_angle = -steering_angle[i]
                print (steering_angle[i], "\n")
                msg.drive.steering_angle_velocity = 0.0
                msg.drive.speed = v1[i]
                print (v1[i], "\n")
                msg.drive.acceleration = 0.0
                print("I pub")
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(0.01)
        print (i, "\n")
        for i in range(60) :
                print("I'm in cycle")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "robot_odom"
                msg.drive.steering_angle = steering_angle_zero[i]
                print (steering_angle_zero[i], "\n")
                msg.drive.steering_angle_velocity = 0.0
                msg.drive.speed = v[i]
                print (v[i], "\n")
                msg.drive.acceleration = 0.0
                print("I pub")
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(0.1)

        print (i, "\n") 
        for i in range(10) :
                print("I'm in cycle")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "robot_odom"
                msg.drive.steering_angle = 0.0
                msg.drive.steering_angle_velocity = 0.0
                msg.drive.speed = 0.0
                msg.drive.acceleration = 0.0
                print("I pub")
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(0.1)

        print (i, "\n") 
        for i in range(70) :
                print("I'm in cycle")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "robot_odom"
                msg.drive.steering_angle = steering_angle_zero[i]
                print (steering_angle_zero[i], "\n")
                msg.drive.steering_angle_velocity = 0.0
                msg.drive.speed = -v[i]
                print (v[i], "\n")
                msg.drive.acceleration = 0.0
                print("I pub")
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(0.1)
        for i in range(100000) :
                print("I'm in cycle")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "robot_odom"
                msg.drive.steering_angle = 0.0
                msg.drive.steering_angle_velocity = 0.0
                msg.drive.speed = 0.0
                msg.drive.acceleration = 0.0
                print("I pub")
                self.cmd_vel_pub.publish(msg)
                rospy.sleep(0.1)

        print (i, "\n")

    def spin(self):
        
        while not rospy.is_shutdown():

            # self.move()
            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(AckermannDriveStamped())
        rospy.sleep(1)

simple_mover = SimpleMover()
# simple_mover.move()
simple_mover.spin()
