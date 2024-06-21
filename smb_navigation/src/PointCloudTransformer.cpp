#include "smb_navigation/PointCloudTransformer.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace smb_navigation {

PointCloudTransformer::PointCloudTransformer(ros::NodeHandle nh) 
    : nh_(nh), tfListener_(tfBuffer_)
{
    // topic names
    ROS_INFO_STREAM("Creating pointcloud transformer ... ...");
    if(!nh_.getParam("global_frame", global_frame_)){
      ROS_ERROR("[PointCloudTransformer] Could not parse the parameter global_frame");
      return;
    }
    if(!nh_.getParam("pcd_topic", pcd_topic_)){
      ROS_ERROR("[PointCloudTransformer] Could not parse the parameter pcd_topic");
      return;
    }
    if(!nh_.getParam("transformed_pcd_topic", transformed_pcd_topic_)){
      ROS_ERROR("[PointCloudTransformer] Could not parse the parameter transformed_pcd_topic");
      return;
    }

    ROS_INFO_STREAM("global_frame_ is " << global_frame_);
    ROS_INFO_STREAM("pcd_topic_ is " << pcd_topic_);
    ROS_INFO_STREAM("transformed_pcd_topic_ is " << transformed_pcd_topic_);

    // subscriber
    pcd_sub_ = nh_.subscribe(pcd_topic_, 1, &PointCloudTransformer::pointCloudCallback, this);

    // publisher
    transformed_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(transformed_pcd_topic_, 1, true);
    ROS_INFO_STREAM("Created pointcloud transformer.");

}

void PointCloudTransformer::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pcd)
{
    if (!tfBuffer_.canTransform(global_frame_, pcd->header.frame_id, pcd->header.stamp, ros::Duration(1.0)))
    {
        ROS_WARN("No transform available from %s to %s at time %f", pcd->header.frame_id.c_str(), global_frame_.c_str(), pcd->header.stamp.toSec());
        return;
    }

    sensor_msgs::PointCloud2 transformed_pcd;
    try
    {
        // Transform the point cloud to the global frame
        tfBuffer_.transform(*pcd, transformed_pcd, global_frame_);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("TF transform exception: %s", ex.what());
        return;
    }
    transformed_pcd_pub_.publish(transformed_pcd);
}

} //namespace smb_navigation