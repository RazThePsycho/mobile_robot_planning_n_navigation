#!/usr/bin/env python3

import time
import itertools
from numpy import sin

import numpy as np
from scipy.spatial.transform import Rotation as R


import rospy
import tf

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult
from actionlib_msgs.msg import GoalID

import cv2
from cv_bridge import CvBridge, CvBridgeError
import camera

from enum import Enum

class GoalStatus(Enum):
    PENDING     = 0 # The goal has yet to be processed by the action server
    ACTIVE      = 1 # The goal is currently being processed by the action server
    PREEMPTED   = 2 # The goal received a cancel request after it started executing
                    #   and has since completed its execution (Terminal State)
    SUCCEEDED   = 3 # The goal was achieved successfully by the action server (Terminal State)
    ABORTED     = 4 # The goal was aborted during execution by the action server due
                    #   to some failure (Terminal State)
    REJECTED    = 5 # The goal was rejected by the action server without being processed,
                    #   because the goal was unattainable or invalid (Terminal State)
    PREEMPTING  = 6 # The goal received a cancel request after it started executing
                    #   and has not yet completed execution
    RECALLING   = 7 # The goal received a cancel request before it started executing,
                    #    but the action server has not yet confirmed that the goal is canceled
    RECALLED    = 8 # The goal received a cancel request before it started executing
                    #    and was successfully cancelled (Terminal State)
    LOST        = 9 # An action client can determine that a goal is LOST. This should not be
                    #    sent over the wire by an action server

class SimpleMover():

    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.sleep(2.0)
        self.detector = camera.Detector()
        self.tf_listener = tf.TransformListener()

        self.goal_pub = rospy.Publisher('/robot/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/robot/robot_cmd_vel', AckermannDriveStamped, queue_size=1)
        self.goal_cancel = rospy.Publisher('/robot/move_base/cancel', GoalID, queue_size=1)

        rospy.Subscriber("/number_of_bags", Int32, self.num_of_bag)
        self.n_bag = 0
        self.n_bag_local = 0

        rospy.Subscriber("/robot/robot_front_ptz_camera/image_raw", Image, self.camera_cb)
        self.cv_image = None
        self.yolo_res = None
        # rospy.Subscriber("/robot/teraranger_duo/sonar", Range, self.sonar)
        # rospy.Subscriber("/robot/lidar_3d/points", PointCloud2, self.lidar)
        rospy.Subscriber("/robot/odometry/filtered/local", Odometry, self.odom)
        self.position = None
        self.orientation = None
        self.velocity = None


        rospy.Subscriber("/robot/move_base/result", MoveBaseActionResult, self.move_base_status)
        self.move_base_status = None
        self.move_base_status_seq = -1
        self.move_base_status_seq_local = -2


        self.rate = rospy.Rate(30)
        self.cv_bridge = CvBridge()
        global gui
        gui = rospy.get_param("/gui", False)

        self._K = np.array([[554.254691191187, 0.0, 320.5],
                            [0.0, 554.254691191187, 240.5],
                            [0.0, 0.0,                1.0]])
        self._cam_w = 640
        self._cam_h = 480
        
        try:
            self.tf_listener.waitForTransform('/world', '/robot_base_footprint', rospy.Time(), rospy.Duration(10.0))
            # robot_front_ptz_camera_optical_frame_link
            trans, rot = self.tf_listener.lookupTransform('/world', '/robot_base_footprint', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        self.go_home_msg = PoseStamped()
        self.go_home_msg.header.frame_id = 'world'
        self.go_home_msg.header.stamp = rospy.Time.now()
        self.go_home_msg.pose.position.x = trans[0]
        self.go_home_msg.pose.position.y = trans[1]
        self.go_home_msg.pose.orientation.x = rot[0]
        self.go_home_msg.pose.orientation.y = rot[1]
        self.go_home_msg.pose.orientation.z = rot[2]
        self.go_home_msg.pose.orientation.w = rot[3]

    def move_base_status(self, msg):
        self.move_base_status = msg.status.status
        self.move_base_status_seq = msg.header.seq
        print(GoalStatus(self.move_base_status))

    def transform_goal(self, visual_goal):
        try:
            self.tf_listener.waitForTransform('/world', '/robot_front_ptz_camera_optical_frame_link', rospy.Time(), rospy.Duration(2.0))
            # robot_front_ptz_camera_optical_frame_link
            trans, rot = self.tf_listener.lookupTransform('/world', '/robot_front_ptz_camera_optical_frame_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        cam_xyz_goal = np.zeros(3)
        cam_xyz_goal[2] = self._K[0, 0]/visual_goal['dpx']
        cam_xyz_goal[0] = cam_xyz_goal[2]*(visual_goal['x'] - self._K[0, 2])/self._K[0, 0]
        cam_xyz_goal[1] = cam_xyz_goal[2]*(visual_goal['y'] - self._K[1, 2])/self._K[1, 1]

        cam_xyz_goal_align = cam_xyz_goal
        cam_xyz_goal_align[1] = 0
        tmp_vec = np.array([1, 0, 0])
        xyz = np.cross(tmp_vec, cam_xyz_goal_align)
        w = np.linalg.norm(cam_xyz_goal_align) + np.dot(tmp_vec, cam_xyz_goal_align)
        direction_rot = R.from_quat([*xyz, w])

        world_cam_rot = R.from_quat(rot)

        global_xyz_goal = trans + world_cam_rot.as_matrix() @ cam_xyz_goal
        global_rot = world_cam_rot * direction_rot
        return (global_xyz_goal, global_rot)

    def send_new_potato_goal(self) -> bool:
        best_res = None
        best_confidence = 0.85
        yolo_res = self.detector.yolo(self.cv_image)
        for res in yolo_res:
            if res['confidence'] > best_confidence:
                best_res = res
                best_confidence = res['confidence']

        if best_res is None:
            return False

        visual_goal = {
            'x': (best_res['xmax']+best_res['xmin'])/2,
            'y': (best_res['ymax']+best_res['ymin'])/2,
            'dpx': best_res['xmax']-best_res['xmin']
        }
        global_goal = self.transform_goal(visual_goal)
        if global_goal is None:
            return False

        global_xyz_goal, global_rot = global_goal
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = global_xyz_goal[0]
        msg.pose.position.y = global_xyz_goal[1]

        q = self.orientation.as_quat()
        # q = global_rot.as_quat()
        # q[2] = 1
        # q = R.from_quat(q).as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.goal_pub.publish(msg)
        return True

    def go_home(self):
        self.goal_pub.publish(self.go_home_msg)

    def turn(self, turn_left: bool = False, r: int = 4):
        if turn_left:
            local_goal = np.array([r, r, 0])
            local_rot = R.from_euler('z', 100, degrees=True)
        else:
            local_goal = np.array([r, -r, 0])
            local_rot = R.from_euler('z', -100, degrees=True)
        global_goal = self.position + self.orientation.as_matrix() @ local_goal
        global_rot = R.from_matrix(self.orientation.as_matrix() @ local_rot.as_matrix())

        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = global_goal[0]
        msg.pose.position.y = global_goal[1]

        q = global_rot.as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.goal_pub.publish(msg)

    def cancel_goals(self):
        msg = GoalID()
        self.goal_cancel.publish(msg)
        print()


    def camera_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cv_image = cv_image
            # self.yolo_res = self.detector.yolo(cv_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

        if gui == True:
            # self.detector.draw_labels(cv_image, self.yolo_res)
            self.show_image(cv_image)
    
    def show_image(self, img):
            cv2.namedWindow("Front Camera Detecter")
            cv2.imshow("Front Camera Detecter", img)
            cv2.waitKey(3)

    def num_of_bag(self, msg):
        self.n_bag = msg.data

    def sonar(self, msg):
        if msg.range < 0.5:
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "robot_base_footprint"
            msg.drive.speed = 4*msg.range - 2
        # print ("Sonar range is: ", msg.range, "\n")

    def lidar(self, msg):
        pass
        # print ("Advertisment to lidar topic. Is dense?", msg.is_dense, "\n")

    def odom(self, msg):
        self.position = np.array([msg.pose.pose.position.x,
                                  msg.pose.pose.position.y,
                                  msg.pose.pose.position.z])
        self.orientation = R.from_quat([msg.pose.pose.orientation.x,
                                        msg.pose.pose.orientation.y,
                                        msg.pose.pose.orientation.z,
                                        msg.pose.pose.orientation.w])
        self.velocity = np.array([msg.twist.twist.linear.x,
                                  msg.twist.twist.linear.y,
                                  msg.twist.twist.linear.z,
                                  msg.twist.twist.angular.x,
                                  msg.twist.twist.angular.y,
                                  msg.twist.twist.angular.z])

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
            if self.n_bag_local < self.n_bag:
                print(self.n_bag_local, self.n_bag_local)
                self.n_bag_local = self.n_bag
                self.cancel_goals()

            if self.move_base_status_seq_local < self.move_base_status_seq:
                if self.n_bag >= 3:
                    self.go_home()
                    print('Go home')
                elif self.send_new_potato_goal():
                    print('Go to potato')
                else:
                    self.turn()
                    print('turn')
                self.move_base_status_seq_local = self.move_base_status_seq

            # self.send_new_potato_goal()
            # self.move()

            
            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(AckermannDriveStamped())
        rospy.sleep(1)

simple_mover = SimpleMover()
# simple_mover.move()
simple_mover.spin()
