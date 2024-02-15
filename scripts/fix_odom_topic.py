#!/usr/bin/env python3

# Author: Vvlad1slavV@yandex.ru

import rospy
from nav_msgs.msg import Odometry

def cmd_callback(data):
    global pub
    pub.publish(data)

if __name__ == '__main__': 
    try:
        rospy.init_node('fix_odom_topic')

        real_odom_topic = rospy.get_param('/real_odom_topic', '/robot/odom')
        correctable_odom_topic = rospy.get_param('/correctable_odom_topic', '/robot/robotnik_base_control/odom')

        rospy.Subscriber(real_odom_topic, Odometry, cmd_callback, queue_size=1)
        pub = rospy.Publisher(correctable_odom_topic, Odometry, queue_size=1)

        rospy.loginfo(f"Node 'fix_odom_topic' started.\nListening to {real_odom_topic}, publishing to {correctable_odom_topic}.")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
