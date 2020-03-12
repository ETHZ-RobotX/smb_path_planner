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
 * path_interceptor.h
 * @brief Header of the path interceptor class
 * @author: Luca Bartolomei
 * Created on: March 12, 2020
 */

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace smb_navigation
{

class PathInterceptor
{

public:
  PathInterceptor(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  ~PathInterceptor() {}

private:
  // Callbacks
  void moveBasePathCallback(const nav_msgs::PathConstPtr& path_msg);

  void odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg);

  // Utility functions
  bool rampInterpolatorWaypoints(
      const std::vector<Eigen::Vector3d>& waypoints,
      std::vector<Eigen::VectorXd>& interpolated_waypoints);

  bool interpolateInitialRotation(
      std::vector<Eigen::VectorXd>& interpolated_waypoints);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber move_base_path_sub_;
  ros::Subscriber odometry_sub_;
  ros::Publisher mpc_path_pub_;

  // Parameters
  double v_max_;
  double a_max_;
  double sampling_dt_;
  double max_initial_rotation_;

  bool has_odometry_;
  Eigen::Vector4d current_state_;

}; // end class PathInterceptor

} // end namespace smb_navigation
