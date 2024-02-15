#include <cmath>
#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Camera from Baggy";

class SimpleMover {

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber image_sub;
    ros::Subscriber sonar_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber gps_sub;
    ros::Rate rate = ros::Rate(30);
    bool gui;
    cv_bridge::CvImagePtr cv_ptr;


  public:

    SimpleMover() {

        image_sub = nh.subscribe("/robot/robot_front_ptz_camera/image_raw", 1, &SimpleMover::camera_cb, this);
        gps_sub = nh.subscribe("/robot/odometry/gps", 1, &SimpleMover::gps_cb, this);
        lidar_sub = nh.subscribe("/robot/lidar_3d/points", 1, &SimpleMover::lidar_cb, this);
        sonar_sub = nh.subscribe("/robot/teraranger_duo/sonar", 1, &SimpleMover::sonar_cb, this);
        cmd_vel_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/robot/robot_cmd_vel", 1);
        // ros::param::get("/gui", gui);
        cv::namedWindow(OPENCV_WINDOW);

        ros::Duration(1).sleep();       // требуется для инициализации времени
    }                                   // при слишком быстром старте узла


    ~SimpleMover() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void camera_cb(const sensor_msgs::Image::ConstPtr &msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if (gui == true)
        {
            show_image(cv_ptr);
        }
        
        
    }
    void gps_cb(const nav_msgs::Odometry::ConstPtr &msg){
        std::cout << "The position of baggy is: " << msg->pose.pose.position.x << msg->pose.pose.position.y << msg->pose.pose.position.z << std::endl;
    }

    void lidar_cb(const sensor_msgs::PointCloud2::ConstPtr &msg){
        std::cout << "Advertise to lidar. Is dense? " << msg->is_dense << std::endl;

    }

    void sonar_cb(const sensor_msgs::Range::ConstPtr &msg){
        std::cout << "Sonar range is " << msg->range << std::endl;
    }

    void show_image(const cv_bridge::CvImagePtr cv_ptr) {
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
    }

    void spin() {
        std::cout << "I'm in spin" << std::endl;
        while (nh.ok()) {
                    std::cout << "I'm in while" << std::endl;
            ackermann_msgs::AckermannDriveStamped twist_msg;
            for (int i = 0; i < 488*6; i++){
                // twist_msg.header.stamp = ros::Time::now();
                twist_msg.header.frame_id = "robot_odom";
                twist_msg.drive.steering_angle = -3.14/8;
                twist_msg.drive.steering_angle_velocity = 0.0;
                twist_msg.drive.speed = 3.0;
                twist_msg.drive.acceleration = 0.0;
                usleep(10000);
            cmd_vel_pub.publish(twist_msg);
            }
            for (int i = 0; i < 950; i++){
                // twist_msg.header.stamp = ros::Time::now();
                twist_msg.header.frame_id = "robot_odom";
                twist_msg.drive.steering_angle = 0.0;
                twist_msg.drive.steering_angle_velocity = 0.0;
                twist_msg.drive.speed = 3.0;
                twist_msg.drive.acceleration = 0.0;
                usleep(10000);
            cmd_vel_pub.publish(twist_msg);
            }
            for (int i = 0; i < 300; i++){
                // twist_msg.header.stamp = ros::Time::now();
                twist_msg.header.frame_id = "robot_odom";
                twist_msg.drive.steering_angle = 0.0;
                twist_msg.drive.steering_angle_velocity = 0.0;
                twist_msg.drive.speed = 0.0;
                twist_msg.drive.acceleration = 0.0;
                usleep(10000);
            cmd_vel_pub.publish(twist_msg);
            }
            for (int i = 0; i < 950; i++){
                // twist_msg.header.stamp = ros::Time::now();
                twist_msg.header.frame_id = "robot_odom";
                twist_msg.drive.steering_angle = 0.0;
                twist_msg.drive.steering_angle_velocity = 0.0;
                twist_msg.drive.speed = -3.0;
                twist_msg.drive.acceleration = 0.0;
                usleep(10000);
            cmd_vel_pub.publish(twist_msg);
            }
            for (int i = 0; i < 5000; i++){
                // twist_msg.header.stamp = ros::Time::now();
                twist_msg.header.frame_id = "robot_odom";
                twist_msg.drive.steering_angle = 0.0;
                twist_msg.drive.steering_angle_velocity = 0.0;
                twist_msg.drive.speed = 0.0;
                twist_msg.drive.acceleration = 0.0;
                usleep(10000);
            cmd_vel_pub.publish(twist_msg);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "simple_mover");

    SimpleMover simpleMover;
    simpleMover.spin();

  return 0;
}
