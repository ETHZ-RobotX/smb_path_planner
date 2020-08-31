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
 * @brief  Node that intercepts the odometry message from the robot and
 *         re-publish the same information as a geometry message of type
 *         PoseStampedWithCovariance (needed by elevation mapping)
 * @date   June 04, 2020
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

/**
 * @brief Main class that implements the interceptor logic
 */
class OdometryInterceptor
{

public:
  OdometryInterceptor(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private)
  {
    odom_sub_ = nh_.subscribe("odometry", 100,
                              &OdometryInterceptor::odometryCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/stamped_pose_covariance", 1, true);
  }

  ~OdometryInterceptor() {}

private:
  void odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header = odom_msg->header;
    pose_msg.pose.pose = odom_msg->pose.pose;
    pose_msg.pose.covariance = odom_msg->pose.covariance;

    pose_pub_.publish(pose_msg);
  }

  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber odom_sub_;
  ros::Publisher pose_pub_;
};

/// @brief Main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_interceptor_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  OdometryInterceptor odometry_interceptor(nh, nh_private);

  ros::spin();
  return 0;
}
