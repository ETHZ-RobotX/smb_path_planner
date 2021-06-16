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
 * rrt_planner.cpp
 * @brief Implementation of the actual solver
 * @author: Luca Bartolomei
 * Created on: March 11, 2020
 */

#include "smb_ompl_planner/rrt_planner.h"

namespace smb_ompl_planner
{

RrtPlanner::RrtPlanner(const RrtParameters& params,
                       const RrtPlannerType& planner_type)
    : params_(params), planner_type_(planner_type)
{
}

RrtPlanner::~RrtPlanner() {}

void RrtPlanner::setupProblem(const Eigen::Vector2d& start,
                              const Eigen::Vector2d& goal)
{
  problem_setup_.clear();

  problem_setup_.setGridmapCollisionChecking(params_.interpolation_factor,
                                             costmap_);

  if (!params_.use_distance_threshold)
  {
    problem_setup_.setDefaultObjective();
  }
  else
  {
    problem_setup_.setLenghtOptimizationObjective(params_.distance_threshold *
                                                  (goal - start).norm());
  }
  if (planner_type_ == kRrtConnect)
  {
    problem_setup_.setRrtConnect();
  }
  else if (planner_type_ == kRrtStar)
  {
    problem_setup_.setRrtStar();
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()->setGoalBias(
        params_.goal_bias);
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()->setRange(
        params_.tree_range);
  }
  else if (planner_type_ == kInformedRrtStar)
  {
    problem_setup_.setInformedRrtStar();
    problem_setup_.getPlanner()->as<ompl::geometric::InformedRRTstar>()->setGoalBias(
        params_.goal_bias);
    problem_setup_.getPlanner()->as<ompl::geometric::InformedRRTstar>()->setRange(
        params_.tree_range);
  }
  else if (planner_type_ == kPrm)
  {
    problem_setup_.setPrm();
  }
  else
  {
    problem_setup_.setDefaultPlanner();
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()->setGoalBias(
        params_.goal_bias);
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()->setRange(
        params_.tree_range);
  }

  if (lower_bound_ != upper_bound_)
  {
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, lower_bound_.x());
    bounds.setLow(1, lower_bound_.y());

    bounds.setHigh(0, upper_bound_.x());
    bounds.setHigh(1, upper_bound_.y());

    // Define start and goal positions.
    problem_setup_.getGeometricComponentStateSpace()
        ->as<ompl::RStateSpace>()
        ->setBounds(bounds);
  }

  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
  double validity_checking_resolution = 0.01;
  if ((upper_bound_ - lower_bound_).norm() > 1e-3)
  {
    // If bounds are set, set this to approximately one voxel.
    validity_checking_resolution =
        voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
  }
  problem_setup_.setStateValidityCheckingResolution(
      validity_checking_resolution);
}

bool RrtPlanner::validStraightLine(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& goal,
                                   const int n_step,
                                   std::vector<Eigen::Vector2d>& path) const
{
  double delta_x = (goal.x() - start.x()) / static_cast<double>(n_step);
  double delta_y = (goal.y() - start.y()) / static_cast<double>(n_step);

  for (int i = 0; i <= n_step; ++i)
  {
    Eigen::Vector2d delta_position(delta_x * static_cast<double>(i),
                                   delta_y * static_cast<double>(i));

    // Check for collision
    std::cout << "Not implemented yet" << std::endl;

    // If no collision is found, then add the position to the storage
    path.push_back(start + delta_position);
  }

  // Add the final position as well - we already know it is valid
  path.push_back(goal);
  return true;
}

bool RrtPlanner::getPathBetweenWaypoints(const Eigen::Vector2d& start,
                                         const Eigen::Vector2d& goal,
                                         std::vector<Eigen::Vector2d>& solution)
{
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  if (problem_setup_.solve(params_.num_seconds_to_plan))
  {
    if (problem_setup_.haveExactSolutionPath())
    {
      if (params_.simplify_solution)
      {
        problem_setup_.reduceVertices();
      }
      if (params_.verbose_planner)
      {
        problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }
    }
    else if (params_.trust_approx_solution && problem_setup_.haveSolutionPath())
    {
      OMPL_WARN("[Ompl Planner] OMPL did not find an exact solution. "
                "Output is an approximate solution! It may be not "
                "feasible for the controller.");
    }
    else
    {
      OMPL_ERROR("[Ompl Planner] OMPL could not find an exact solution.");
      return false;
    }
  }

  if (problem_setup_.haveSolutionPath())
  {
    solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(), solution);
    return true;
  }
  return false;
}

void RrtPlanner::setupFromStartAndGoal(const Eigen::Vector2d& start,
                                       const Eigen::Vector2d& goal)
{

  if (planner_type_ == kPrm)
  {
    std::dynamic_pointer_cast<ompl::geometric::PRM>(problem_setup_.getPlanner())
        ->clearQuery();
  }
  else
  {
    problem_setup_.clear();
  }

  ompl::base::ScopedState<ompl::RStateSpace> start_ompl(
      problem_setup_.getSpaceInformation());
  ompl::base::ScopedState<ompl::RStateSpace> goal_ompl(
      problem_setup_.getSpaceInformation());

  start_ompl->values[0] = start.x();
  start_ompl->values[1] = start.y();

  goal_ompl->values[0] = goal.x();
  goal_ompl->values[1] = goal.y();

  problem_setup_.setStartAndGoalStates(start_ompl, goal_ompl,
                                       0.5 * voxel_size_);
  problem_setup_.setup();

  // Add optimization objective
  if (params_.use_distance_threshold)
  {
    problem_setup_.setLenghtOptimizationObjective(params_.distance_threshold *
                                                  (goal - start).norm());
  }
}

void RrtPlanner::solutionPathToTrajectoryPoints(
    ompl::geometric::PathGeometric& path,
    std::vector<Eigen::Vector2d>& trajectory_points) const
{
  trajectory_points.clear();
  trajectory_points.reserve(path.getStateCount());

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector)
  {
    Eigen::Vector2d robot_position(
        state_ptr->as<ompl::RStateSpace::StateType>()->values[0],
        state_ptr->as<ompl::RStateSpace::StateType>()->values[1]);

    trajectory_points.emplace_back(robot_position);
  }
}

bool RrtPlanner::getBestPathTowardGoal(const Eigen::Vector2d& start,
                                       const Eigen::Vector2d& goal,
                                       std::vector<Eigen::Vector2d>& solution)
{
  solution.clear();
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  bool solution_found = false;
  solution_found = problem_setup_.solve(params_.num_seconds_to_plan);
  if (solution_found)
  {
    if (problem_setup_.haveSolutionPath())
    {
      // Simplify and print.
      if (params_.simplify_solution)
      {
        problem_setup_.reduceVertices();
      }
      if (params_.verbose_planner)
      {
        problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }
      solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(),
                                     solution);
      return true;
    }
  }
  // The case where you actually have a solution path has returned by now.
  // Otherwise let's just see what the best we can do is.
  ompl::base::PlannerData planner_data(problem_setup_.getSpaceInformation());
  problem_setup_.getPlanner()->getPlannerData(planner_data);

  // Start traversing the graph and find the node that gets the closest to the
  // actual goal point.
  if (planner_data.numStartVertices() < 1)
  {
    OMPL_ERROR("No start vertices in RRT!");
    return false;
  }

  unsigned int min_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  if (planner_data.numVertices() <= 0)
  {
    OMPL_ERROR("No vertices in RRT!");
    return false;
  }

  // Iterate over all vertices. Check which is the closest.
  for (unsigned int i = 0; i < planner_data.numVertices(); i++)
  {
    const ompl::base::PlannerDataVertex& vertex = planner_data.getVertex(i);
    double distance = getDistanceEigenToState(goal, vertex.getState());

    if (distance < min_distance)
    {
      min_distance = distance;
      min_index = i;
    }
  }

  unsigned int start_index = planner_data.getStartIndex(0);

  // Get the closest vertex back out, and then get its parents.
  std::vector<Eigen::Vector2d> trajectory_points;

  unsigned int current_index = min_index;
  while (current_index != start_index)
  {
    // Put this vertex in.
    const ompl::base::PlannerDataVertex& vertex =
        planner_data.getVertex(current_index);

    const ompl::base::State* state_ptr = vertex.getState();
    Eigen::Vector2d robot_position(
        state_ptr->as<ompl::RStateSpace::StateType>()->values[0],
        state_ptr->as<ompl::RStateSpace::StateType>()->values[1]);
    trajectory_points.emplace_back(robot_position);

    std::vector<unsigned int> edges;
    planner_data.getIncomingEdges(current_index, edges);

    if (edges.empty())
    {
      break;
    }
    current_index = edges.front();
  }

  // Finally reverse the vector.
  std::reverse(std::begin(trajectory_points), std::end(trajectory_points));
  solution = trajectory_points;
  return false;
}

double RrtPlanner::getDistanceEigenToState(const Eigen::Vector2d& eigen,
                                           const ompl::base::State* state_ptr)
{
  Eigen::Vector2d state_pos(
      state_ptr->as<ompl::RStateSpace::StateType>()->values[0],
      state_ptr->as<ompl::RStateSpace::StateType>()->values[1]);

  return (eigen - state_pos).norm();
}

} // end namespace smb_ompl_planner

