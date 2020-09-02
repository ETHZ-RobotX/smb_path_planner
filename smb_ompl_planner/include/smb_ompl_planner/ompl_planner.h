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
 * ompl_planner.h
 * @brief Header file for the main class
 * @author: Luca Bartolomei
 * Created on: March 11, 2020
 */

#pragma once

#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "smb_ompl_planner/orientation_filter.h"
#include "smb_ompl_planner/rrt_planner.h"

namespace smb_ompl_planner
{

class OmplPlanner : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief  Default constructor for the OmplPlanner object
   */
  OmplPlanner();

  /**
   * @brief  Constructor for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  frame_id Frame of the costmap
   */
  OmplPlanner(std::string name, costmap_2d::Costmap2D* costmap,
              std::string frame_id);

  /**
   * @brief  Default deconstructor for the PlannerCore object
   */
  ~OmplPlanner();

  /**
   * @brief  Initialization function for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for
   * planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2D* costmap,
                  std::string frame_id);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance The tolerance on the goal point for the planner
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, const double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);
  /**
   * @brief  Publish a path for visualization purposes
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

  /**
   * @brief Callbacks
   */
  bool makePlanService(nav_msgs::GetPlan::Request& req,
                       nav_msgs::GetPlan::Response& resp);
  void odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void collisionTimerCallback(const ros::TimerEvent&);

protected:
  /**
   * @brief Store a copy of the current costmap in \a costmap.  Called by
   * makePlan.
   */
  costmap_2d::Costmap2D* costmap_;
  std::string frame_id_;

  ros::Publisher plan_pub_;
  ros::Subscriber odometry_sub_;
  ros::Timer timer_collisions_;

  bool initialized_;
  bool has_odometry_;

  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      global_path_;
  Eigen::Vector2d odometry_;
  Eigen::Vector2d goal_;

private:
  void mapToWorld(double mx, double my, double& wx, double& wy);
  bool worldToMap(double wx, double wy, double& mx, double& my);
  void clearRobotCell(const geometry_msgs::PoseStamped& global_pose,
                      unsigned int mx, unsigned int my);
  void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

  // Planner
  std::shared_ptr<smb_ompl_planner::RrtPlanner> ompl_planner_;
  std::shared_ptr<OrientationFilter> orientation_filter_;

  // Utilities
  boost::mutex mutex_;
  ros::ServiceServer make_plan_srv_;

  unsigned char* cost_array_;
  unsigned int start_x_, start_y_, end_x_, end_y_;

  double default_tolerance_;
  double min_distance_waypoints_;
  double dist_goal_reached_;

  bool old_navfn_behavior_;
  float convert_offset_;
}; // end class OmplPlanner

} // end namespace smb_ompl_planner
