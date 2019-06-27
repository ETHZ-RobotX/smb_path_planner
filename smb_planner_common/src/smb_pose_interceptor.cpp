/**
  * @author Luca Bartolomei, V4RL
  * @brief  Message interceptor to get the right message type for traversability
  *         estimation.
  * @date   18.06.2019
  */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <smb_msgs/SmbState.h>

/**
  * @brief Main class that implements the interceptor logic
  */
class PoseInterceptor {

public:
  PoseInterceptor(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
      : nh_(nh), nh_private_(nh_private) {
    pose_sub_ = nh_.subscribe("/base_pose_measured", 100,
                              &PoseInterceptor::poseCallback, this);
    smb_state_sub_ = nh_.subscribe("/state_estimator/smb_state", 100,
                                   &PoseInterceptor::smbStateCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/stamped_pose_covariance", 1, true);
  }

private:
  void poseCallback(const geometry_msgs::PoseStamped &msg) {

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = msg.header;
    pose_msg.pose.pose = msg.pose;
    pose_msg.pose.covariance[0] = 0.01;
    pose_msg.pose.covariance[7] = 0.01;
    pose_msg.pose.covariance[14] = 0.01;
    pose_msg.pose.covariance[21] = 0.0;
    pose_msg.pose.covariance[28] = 0.0;
    pose_msg.pose.covariance[35] = 0.01;

    pose_pub_.publish(pose_msg);
  }

  void smbStateCallback(const smb_msgs::SmbState &state_msg) {

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = state_msg.header;
    pose_msg.pose.pose = state_msg.pose.pose;
    pose_msg.pose.covariance[0] = 0.01;
    pose_msg.pose.covariance[7] = 0.01;
    pose_msg.pose.covariance[14] = 0.01;
    pose_msg.pose.covariance[21] = 0.0;
    pose_msg.pose.covariance[28] = 0.0;
    pose_msg.pose.covariance[35] = 0.01;

    pose_pub_.publish(pose_msg);
  }

  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber pose_sub_;
  ros::Subscriber smb_state_sub_;
  ros::Publisher pose_pub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "smb_pose_interceptor");
  ROS_INFO("SMB Pose interceptor started");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private;

  PoseInterceptor pose_interceptor = PoseInterceptor(nh, nh_private);
  ros::spin();
  return 0;
}
