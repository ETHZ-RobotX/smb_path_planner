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
 * ompl_planner_node.cpp
 * @brief Main node for the planner
 * @author: Luca Bartolomei
 * Created on: March 11, 2020
 */

#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/MakeNavPlan.h>
#include <tf2_ros/transform_listener.h>

#include "smb_ompl_planner/ompl_planner.h"

using namespace costmap_2d;

namespace smb_ompl_planner
{

class OmplPlannerWithCostmap : public OmplPlanner
{

public:
  OmplPlannerWithCostmap(std::string name, Costmap2DROS* cmap);
  bool makePlanService(navfn::MakeNavPlan::Request& req,
                       navfn::MakeNavPlan::Response& resp);

private:
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
  Costmap2DROS* cmap_;
  ros::ServiceServer make_plan_service_;
  ros::Subscriber pose_sub_;
};

bool OmplPlannerWithCostmap::makePlanService(navfn::MakeNavPlan::Request& req,
                                             navfn::MakeNavPlan::Response& resp)
{
  std::vector<geometry_msgs::PoseStamped> path;

  req.start.header.frame_id = "map";
  req.goal.header.frame_id = "map";
  bool success = makePlan(req.start, req.goal, path);
  resp.plan_found = success;
  if (success)
  {
    resp.path = path;
  }

  return true;
}

void OmplPlannerWithCostmap::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  geometry_msgs::PoseStamped global_pose;
  cmap_->getRobotPose(global_pose);
  std::vector<geometry_msgs::PoseStamped> path;
  makePlan(global_pose, *goal, path);
}

OmplPlannerWithCostmap::OmplPlannerWithCostmap(std::string name,
                                               Costmap2DROS* cmap)
    : OmplPlanner(name, cmap->getCostmap(), cmap->getGlobalFrameID())
{
  ros::NodeHandle private_nh("~");
  cmap_ = cmap;
  make_plan_service_ = private_nh.advertiseService(
      "make_plan", &OmplPlannerWithCostmap::makePlanService, this);
  pose_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>(
      "goal", 1, &OmplPlannerWithCostmap::poseCallback, this);
}

} // end namespace smb_ompl_planner

/******************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "smb_ompl_planner");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS lcr("costmap", buffer);
  smb_ompl_planner::OmplPlannerWithCostmap ompl_planner_with_costmap(
      "ompl_planner", &lcr);

  ros::spin();
  return 0;
}
