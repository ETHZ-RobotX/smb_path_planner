/*
 * Transform the odom frame-child_frame1 to odom frame-child_frame2
 * 
 */
#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace smb_navigation {

class OdometryTransformer
{
private:
    // nodehandle
    ros::NodeHandle nh_;
    
    // subsriber
    ros::Subscriber odom_sub_;

    // publisher
    ros::Publisher transformed_odom_pub_;

    // tf
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;  

    // topics
    std::string odom_topic_;
    std::string transformed_odom_topic_;

    // frames
    std::string target_frame_;

    // callbacks
    void odometryCallback(const nav_msgs::OdometryConstPtr& odom);

public:
    OdometryTransformer(ros::NodeHandle nh);
};

}