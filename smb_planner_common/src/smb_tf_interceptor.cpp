/**
  * @author Luca Bartolomei, V4RL
  * @brief  Message interceptor to get the right message type for traversability
  *         estimation.
  * @date   26.09.2019
  */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

/**
  * @brief Main class that implements the interceptor logic
  */
class TfInterceptor {

public:
  TfInterceptor(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
      : nh_(nh), nh_private_(nh_private), world_frame_("world"), 
        robot_frame_("body") {
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/stamped_pose_covariance", 1, true);
        
    // Parameters
    std::string ns("smb_tf_interceptor_node/");
    if(!nh_.getParam(ns + "world_frame", world_frame_)) {
      ROS_WARN("[TF Interceptor] World frame name not specified");
    }    
    
    if(!nh_.getParam(ns + "robot_frame", robot_frame_)) {
      ROS_WARN("[TF Interceptor] Robot frame name not specified");
    }    
        
    // Setup tf listener    
    tf::TransformListener listener;    
    ros::Rate rate(10.0); 
    int iter = 0;
       
    while (nh_.ok()) {
      tf::StampedTransform transform;
      try {
        listener.lookupTransform(world_frame_, robot_frame_,  
                                 ros::Time(0), transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR_THROTTLE(10, "%s",ex.what());
        ros::Duration(0.05).sleep();
      }
      
      // Create the message to publish
      geometry_msgs::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.frame_id = world_frame_;
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.seq = iter;
      
      pose_msg.pose.pose.position.x = transform.getOrigin().getX();
      pose_msg.pose.pose.position.y = transform.getOrigin().getY();
      pose_msg.pose.pose.position.z = transform.getOrigin().getZ();
      
      pose_msg.pose.pose.orientation.x = transform.getRotation().getX();
      pose_msg.pose.pose.orientation.y = transform.getRotation().getY();
      pose_msg.pose.pose.orientation.z = transform.getRotation().getZ();
      pose_msg.pose.pose.orientation.w = transform.getRotation().getW();
      
      pose_msg.pose.covariance[0] = 0.01;
      pose_msg.pose.covariance[7] = 0.01;
      pose_msg.pose.covariance[14] = 0.01;
      pose_msg.pose.covariance[21] = 0.0;
      pose_msg.pose.covariance[28] = 0.0;
      pose_msg.pose.covariance[35] = 0.01;
    
      pose_pub_.publish(pose_msg);
      rate.sleep();
      ++iter;
    }    
  }

private:
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber pose_sub_;
  ros::Publisher pose_pub_;
  
  std::string world_frame_;
  std::string robot_frame_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "smb_tf_interceptor");
  ROS_INFO("SMB TF interceptor started");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private;

  TfInterceptor tf_interceptor = TfInterceptor(nh, nh_private);
  ros::spin();
  return 0;
}
