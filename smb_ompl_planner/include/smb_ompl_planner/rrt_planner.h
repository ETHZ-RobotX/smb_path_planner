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
 * rrt_planner.h
 * @brief Header file for the actual solver
 * @author: Luca Bartolomei
 * Created on: March 11, 2020
 */

#include "smb_ompl_planner/ompl_setup.h"

namespace smb_ompl_planner
{

/**
 * @brief List of available algorithms
 */
enum RrtPlannerType
{
  kRrtConnect = 0,
  kRrtStar,
  kInformedRrtStar,
  kPrm
};

/**
 * @brief Parameters for the planner
 */
struct RrtParameters
{
  bool use_distance_threshold;
  bool simplify_solution;
  bool trust_approx_solution;
  bool verbose_planner;

  double distance_threshold;
  double goal_bias;
  double tree_range;
  double interpolation_factor;
  double num_seconds_to_plan;
};

class RrtPlanner
{

public:
  /**
   * @brief Constructor
   */
  RrtPlanner(const RrtParameters& params,
             const RrtPlannerType& planner_type = RrtPlannerType::kRrtStar);

  /**
   * @brief Destructor
   */
  ~RrtPlanner();

  /**
   * @brief Method to set up the planning problem from start to goal
   * @param[in] start: start position of the robot
   * @param[in] goal: goal to go to
   */
  void setupProblem(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);

  /**
   * @brief Bunch of getter and setter
   */
  void setBounds(const Eigen::Vector2d& lower_bound,
                 const Eigen::Vector2d& upper_bound)
  {
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  void setVoxelSize(const double voxel_size) { voxel_size_ = voxel_size; }

  void setCostmap(costmap_2d::Costmap2D* costmap) { costmap_ = costmap; }

  /**
   * @brief Method to get the straight line planning between start and goal
   * @param[in] start: start position of the robot
   * @param[in] goal: goal to go to
   * @param[in] n_step: number of steps to be checked along straight line
   * @param[out] path: sequence of positions along line
   * @return True if straight line is possible, False otherwise
   */
  bool validStraightLine(const Eigen::Vector2d& start,
                         const Eigen::Vector2d& goal, const int n_step,
                         std::vector<Eigen::Vector2d>& path) const;

  // Fixed start and end locations, returns list of waypoints between.
  bool getPathBetweenWaypoints(const Eigen::Vector2d& start,
                               const Eigen::Vector2d& goal,
                               std::vector<Eigen::Vector2d>& solution);

  void solutionPathToTrajectoryPoints(
      ompl::geometric::PathGeometric& path,
      std::vector<Eigen::Vector2d>& trajectory_points) const;

  // Even if planning fails, get the part of the tree that spans closest to
  // the original goal point. Returns true if it was actually successfully
  // able to plan to the original goal point, false otherwise.
  bool getBestPathTowardGoal(const Eigen::Vector2d& start,
                             const Eigen::Vector2d& goal,
                             std::vector<Eigen::Vector2d>& solution);

protected:
  /**
   * @brief Method to setup the problem using the start and goal information
   * @param[in] start: starting position for the planning
   * @param[in] goal: goal position
   */
  void setupFromStartAndGoal(const Eigen::Vector2d& start,
                             const Eigen::Vector2d& goal);

  /**
   * @brief Utility function to get the distance between a state in ompl format
   *        and a state in eigen format
   * @return Distance value
   */
  double getDistanceEigenToState(const Eigen::Vector2d& eigen,
                                 const ompl::base::State* state_ptr);

private:
  // Set up the problem in OMPL
  ompl::OmplSetup problem_setup_;

  // Costmap
  costmap_2d::Costmap2D* costmap_;

  // Limits of the planning
  Eigen::Vector2d lower_bound_;
  Eigen::Vector2d upper_bound_;

  // Parameters
  RrtParameters params_;
  RrtPlannerType planner_type_;
  double voxel_size_;
};

} // end namespace smb_ompl_planner

