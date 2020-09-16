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
 * path_interceptor.cpp
 * @brief Implementation of the path interceptor class
 * @author: Luca Bartolomei
 * Created on: March 12, 2020
 */

#include "smb_navigation/path_interceptor.h"

#include <tf/tf.h>

namespace smb_navigation
{

PathInterceptor::PathInterceptor(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), has_odometry_(false)
{
  // Read parameters
  if (!nh_private_.getParam("v_max", v_max_))
  {
    ROS_WARN("[Path Interceptor] V Max not specified. Using 1.0 m/s");
    v_max_ = 1.0;
  }

  if (!nh_private_.getParam("a_max", a_max_))
  {
    ROS_WARN("[Path Interceptor] A Max not specified. Using 1.0 m/s2");
    a_max_ = 1.0;
  }

  if (!nh_private_.getParam("sampling_dt", sampling_dt_))
  {
    ROS_WARN("[Path Interceptor] Sampling Dt not specified. Using 0.05 s");
    sampling_dt_ = 0.05;
  }

  if (!nh_private_.getParam("max_initial_rotation", max_initial_rotation_))
  {
    ROS_WARN(
        "[Path Interceptor] Max initial rotation not specified. Using 45 deg");
    max_initial_rotation_ = M_PI / 4.0;
  }
  else
  {
    max_initial_rotation_ *= M_PI / 180.0;
  }

  // Initialize ROS communication
  move_base_path_sub_ = nh_.subscribe(
      "move_base/path", 10,
      &smb_navigation::PathInterceptor::moveBasePathCallback, this);
  odometry_sub_ = nh_.subscribe(
      "odometry", 10, &smb_navigation::PathInterceptor::odometryCallback, this);

  mpc_path_pub_ = nh_.advertise<nav_msgs::Path>("mpc_trajectory", 100);
}

void PathInterceptor::moveBasePathCallback(
    const nav_msgs::PathConstPtr& path_msg)
{
  if (mpc_path_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  if (!has_odometry_)
  {
    ROS_ERROR("[Path Interceptor] No odometry yet");
    return;
  }

  // Get the positions from the path
  std::vector<Eigen::Vector3d> waypoints;
  for (auto pose : path_msg->poses)
  {
    waypoints.push_back(Eigen::Vector3d(
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  }

  // Interpolate the path
  std::vector<Eigen::VectorXd> interpolated_waypoints;
  if (!rampInterpolatorWaypoints(waypoints, interpolated_waypoints))
  {
    ROS_ERROR("[Path Interceptor] Interpolation of waypoints failed");
    return;
  }

  // Now check if we need to add interpolation of the rotation at the beginning
  // of the path
  //  if (interpolateInitialRotation(interpolated_waypoints))
  //  {
  //    ROS_INFO("[Path Interceptor] Interpolated initial rotation");
  //  }

  // Now store the interpolated path and publish it
  nav_msgs::Path interpolated_path_msg;
  interpolated_path_msg.header = path_msg->header;
  interpolated_path_msg.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = interpolated_path_msg.header;
  pose_stamped.header.seq = 0;

  // In order to avoid trying to track a bad path, we skip the first waypoint
  bool skip_first = true;
  double initial_time = interpolated_path_msg.header.stamp.toSec();
  for (auto wp : interpolated_waypoints)
  {
    if (skip_first)
    {
      skip_first = false;
      continue;
    }

    pose_stamped.header.stamp = ros::Time(initial_time + wp(4));
    pose_stamped.pose.position.x = wp(0);
    pose_stamped.pose.position.y = wp(1);
    pose_stamped.pose.position.z = wp(2);
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(wp(3));

    interpolated_path_msg.poses.push_back(pose_stamped);
    pose_stamped.header.seq++;
  }

  mpc_path_pub_.publish(interpolated_path_msg);
}

void PathInterceptor::odometryCallback(
    const nav_msgs::OdometryConstPtr& odom_msg)
{
  has_odometry_ = true;
  current_state_(0) = odom_msg->pose.pose.position.x;
  current_state_(1) = odom_msg->pose.pose.position.y;
  current_state_(2) = odom_msg->pose.pose.position.z;
  current_state_(3) = tf::getYaw(odom_msg->pose.pose.orientation);
}

bool PathInterceptor::rampInterpolatorWaypoints(
    const std::vector<Eigen::Vector3d>& waypoints,
    std::vector<Eigen::VectorXd>& interpolated_waypoints)
{

  // The waypoint list is empty --> unsuccessful interpolation
  if (waypoints.empty())
  {
    return false;
  }

  // Initialize variables
  double time = 0.0;
  double velocity = 0.0;

  // Iterate over pair of waypoints
  for (size_t i = 0; i < waypoints.size() - 1; ++i)
  {
    // Extract start and end point for the current segment
    Eigen::Vector3d start = waypoints[i];
    Eigen::Vector3d end = waypoints[i + 1];

    // Extract the current yaw (x,y plane)
    Eigen::Vector2d direction = (end - start).head(2).normalized();
    double yaw = std::atan2(direction.y(), direction.x());

    // Figure out what the total segment time will be.
    double total_segment_distance = (end - start).norm();
    // Total time needed to get to max speed (or go from max speed to 0)
    double min_acceleration_time = v_max_ / a_max_;
    // The amount of distance covered during the acceleration
    // (or decceleration process).
    double min_acceleration_distance =
        v_max_ * min_acceleration_time -
        0.5 * a_max_ * min_acceleration_time * min_acceleration_time;

    double total_segment_time = 0.0;
    // Case 1: time is shorter than the acceleration and
    // decceleration time.
    if (total_segment_distance < 2 * min_acceleration_distance)
    {
      total_segment_time = 2 * std::sqrt(total_segment_distance / a_max_);
    }
    else
    {
      // Case 2: time is longer than accel + deccel time.
      total_segment_time =
          2 * min_acceleration_time +
          (total_segment_distance - 2 * min_acceleration_distance) / v_max_;
    }
    size_t num_elements = total_segment_time / sampling_dt_;
    Eigen::Vector3d path_direction = (end - start).normalized();

    // Treat this as a 1D problem since it is. ;)
    double position = 0.0;

    // Separate the time between total time and local segment time
    double current_time_trajectory = time;
    int64_t current_time_segment = 0.0;

    for (size_t j = 0; j < num_elements; ++j)
    {
      // Integrate velocity to get position.
      position += velocity * sampling_dt_;

      // Figure out if we're accelerating, deccelerating, or neither.
      // Handle Case 1 first:
      if (total_segment_time < min_acceleration_time * 2)
      {
        if (current_time_segment < total_segment_time / 2.0)
        {
          velocity += a_max_ * sampling_dt_;
        }
        else
        {
          velocity -= a_max_ * sampling_dt_;
        }
      }
      else
      {
        // Case 2
        if (position <= min_acceleration_distance)
        {
          velocity += a_max_ * sampling_dt_;
        }
        else if ((total_segment_distance - position) <=
                 min_acceleration_distance)
        {
          velocity -= a_max_ * sampling_dt_;
        }
      }

      // Make sure to meet constraints (could be passed/missed due to
      // discretization error).
      if (position > total_segment_distance)
      {
        position = total_segment_distance;
      }
      if (velocity > v_max_)
      {
        velocity = v_max_;
      }
      if (velocity < 0)
      {
        velocity = 0;
      }

      // Save the interpolated path
      Eigen::Vector3d posit(start + path_direction * position);
      Eigen::VectorXd point(5);
      point << posit.x(), posit.y(), posit.z(), yaw, current_time_trajectory;
      interpolated_waypoints.push_back(point);

      // Update times
      current_time_trajectory += sampling_dt_;
      current_time_segment += sampling_dt_;
    }

    time = current_time_trajectory + sampling_dt_;
  }

  // Add end point
  Eigen::Vector3d position_end = waypoints.back();
  Eigen::Vector2d direction_end =
      (position_end.head(2) - waypoints[waypoints.size() - 2].head(2))
          .normalized();
  double yaw_end = std::atan2(direction_end.y(), direction_end.x());

  Eigen::VectorXd point_end(5);
  point_end << position_end.x(), position_end.y(), position_end.z(), yaw_end,
      time;
  interpolated_waypoints.push_back(point_end);
  return true;
}

bool PathInterceptor::interpolateInitialRotation(
    std::vector<Eigen::VectorXd>& interpolated_waypoints)
{
  if (interpolated_waypoints.size() < 2)
  {
    return false;
  }

  Eigen::Vector2d direction_rot =
      (interpolated_waypoints[1].head<2>() - current_state_.head<2>())
          .normalized();
  double target_yaw = std::atan2(direction_rot(1), direction_rot(0));

  // Check if we need to add rotation; if not, just return
  if (std::fabs(current_state_(3) - target_yaw) >= max_initial_rotation_)
  {

    double yaw = current_state_(3);
    double time = 0.0;

    // Create vector of initial rotation commands
    double delta = target_yaw - current_state_(3);
    if (delta > M_PI)
      delta -= 2.0 * M_PI;
    if (delta < -M_PI)
      delta += 2.0 * M_PI;
    size_t num_elements = std::fabs(delta) / (v_max_ * sampling_dt_);
    std::vector<Eigen::VectorXd> rotation_vector(num_elements);

    for (size_t i = 0; i < num_elements; ++i)
    {
      yaw += delta / num_elements;
      if (yaw > 2.0 * M_PI)
        yaw -= 2.0 * M_PI;
      if (yaw < -2.0 * M_PI)
        yaw += 2.0 * M_PI;

      Eigen::VectorXd waypoint(5);
      waypoint << current_state_(0), current_state_(1), current_state_(2), yaw,
          time;
      rotation_vector[i] = waypoint;
      time += sampling_dt_;
    }

    if (!rotation_vector.empty())
    { // ie num_elements != 0
      // Insert the rotation command vector to the beginning of the interpolated
      // waypoints vector
      interpolated_waypoints.insert(interpolated_waypoints.begin(),
                                    rotation_vector.begin(),
                                    rotation_vector.end());
      // Re-timing of the interpolated waypoints to account for new commands of
      // initial rotation (i.e. shift all the timings by the initial offset)
      for (size_t i = num_elements; i < interpolated_waypoints.size(); ++i)
      {
        interpolated_waypoints[i](4) +=
            rotation_vector.back()(4) + sampling_dt_;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

} // end namespace smb_navigation
