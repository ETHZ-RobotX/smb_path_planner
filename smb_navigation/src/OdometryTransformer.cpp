#include "smb_navigation/OdometryTransformer.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace smb_navigation {

OdometryTransformer::OdometryTransformer(ros::NodeHandle nh) 
    : nh_(nh), tfListener_(tfBuffer_)
{
    // topic names
    ROS_INFO_STREAM("Creating odometry transformer ... ...");
    if(!nh_.getParam("target_frame", target_frame_)){
      ROS_ERROR("[OdometryTransformer] Could not parse the parameter target_frame");
      return;
    }
    if(!nh_.getParam("odom_topic", odom_topic_)){
      ROS_ERROR("[OdometryTransformer] Could not parse the parameter odom_topic");
      return;
    }
    if(!nh_.getParam("transformed_odom_topic", transformed_odom_topic_)){
      ROS_ERROR("[OdometryTransformer] Could not parse the parameter transformed_odom_topic");
      return;
    }

    ROS_INFO_STREAM("target_frame_ is " << target_frame_);
    ROS_INFO_STREAM("odom_topic is " << odom_topic_);
    ROS_INFO_STREAM("transformed_odom_topic_ is " << transformed_odom_topic_);

    // subscriber
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &OdometryTransformer::odometryCallback, this);

    // publisher
    transformed_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(transformed_odom_topic_, 1, true);
    ROS_INFO_STREAM("Created odometry transformer.");

}

void OdometryTransformer::odometryCallback(const nav_msgs::OdometryConstPtr& odom)
{
    nav_msgs::Odometry transformed_odom = *odom;
    try {
        geometry_msgs::TransformStamped transformStamped = tfBuffer_.lookupTransform(odom->child_frame_id, target_frame_,
                                                                                            odom->header.stamp, 
                                                                                            ros::Duration(1.0));
        // Transform the input odom to target_frame_ pose in odom frame
        tf2::doTransform(odom->pose.pose, transformed_odom.pose.pose, transformStamped);

        // Publish the transformed odometry message
        transformed_odom.child_frame_id = target_frame_;
        transformed_odom_pub_.publish(transformed_odom);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
}

} //namespace smb_navigation