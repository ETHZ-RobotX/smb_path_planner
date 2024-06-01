#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace smb_navigation {

class PointCloudTransformer
{
private:
    // nodehandle
    ros::NodeHandle nh_;
    
    // subsriber
    ros::Subscriber pcd_sub_;

    // publisher
    ros::Publisher transformed_pcd_pub_;

    // tf
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;  

    // topics
    std::string pcd_topic_;
    std::string transformed_pcd_topic_;

    // frames
    std::string global_frame_;

    // callbacks
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pcd);

public:
    PointCloudTransformer(ros::NodeHandle nh);
};

}