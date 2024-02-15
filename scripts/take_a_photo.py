#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

def save_picture():
    cv_bridge = CvBridge()
    try:
        image_frame = cv_bridge.imgmsg_to_cv2(
            rospy.wait_for_message("/robot/robot_front_ptz_camera/image_raw", Image), 
            'bgr8'
        )
        cv2.imwrite("/ros_wd/src/bachelor/images/front.jpg", image_frame)
        rospy.loginfo("Image was saved")
    except Exception as e:
        rospy.logerr('Error: imwrite...', e)

if __name__ == '__main__':
    save_picture()
