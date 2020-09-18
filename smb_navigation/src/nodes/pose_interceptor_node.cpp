/*
 * Copyright (c) 2020, Vision for Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * @author Luca Bartolomei, V4RL
 * @brief  Node that intercepts the pose stamped message from Gazebo and
 *         re-publish the same information as a odometry message needed by
 *         move_base
 * @date   September 18, 2020
 */

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

/**
 * @brief Main class that implements the interceptor logic
 */
class PoseStampedInterceptor
{

public:
  PoseStampedInterceptor(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private)
  {
    pose_sub_ =
        nh_.subscribe("/base_pose_measured", 100,
                      &PoseStampedInterceptor::poseStampedCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odometry", 1, true);

    if (!nh_private_.getParam("frame_id", frame_id_))
    {
      ROS_WARN("[Pose Interceptor] Frame ID not specified. Using 'world'");
      frame_id_ = "world";
    }
  }

  ~PoseStampedInterceptor() {}

private:
  void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg)
  {
    nav_msgs::Odometry odom_msg;
    odom_msg.header = pose_msg->header;
    odom_msg.header.frame_id = frame_id_;
    odom_msg.pose.pose = pose_msg->pose;

    odom_pub_.publish(odom_msg);
  }

  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber pose_sub_;
  ros::Publisher odom_pub_;

  std::string frame_id_;
};

/// @brief Main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_stamped_interceptor_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  PoseStampedInterceptor odometry_interceptor(nh, nh_private);

  ros::spin();
  return 0;
}
