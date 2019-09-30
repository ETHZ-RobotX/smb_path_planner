/*
 * ompl_voxblox.h
 * @brief Source file for implementation for ompl rrt using voxblox
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: Jan 23, 2019
 */

#include "smb_global_planner/planner/voxblox_ompl_rrt.h"

namespace smb_global_planner {

VoxbloxOmplRrt::VoxbloxOmplRrt(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private,
                               const PlannerParameters &params)
    : nh_(nh), nh_private_(nh_private), params_(params),
      planner_type_(kRrtStar), optimistic_(false), check_traversability_(false),
      lower_bound_(Eigen::Vector3d::Zero()),
      upper_bound_(Eigen::Vector3d::Zero()) {}

void VoxbloxOmplRrt::setBounds(const Eigen::Vector3d &lower_bound,
                               const Eigen::Vector3d &upper_bound) {
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

void VoxbloxOmplRrt::setTsdfLayer(
    voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer) {
  tsdf_layer_ = tsdf_layer;
  CHECK_NOTNULL(tsdf_layer_);
  voxel_size_ = tsdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setEsdfLayer(
    voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
  esdf_layer_ = esdf_layer;
  CHECK_NOTNULL(esdf_layer_);
  voxel_size_ = esdf_layer_->voxel_size();
}

void VoxbloxOmplRrt::setupProblem(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &goal) {
  problem_setup_.clear();

  if (optimistic_) {
    CHECK_NOTNULL(tsdf_layer_);
    problem_setup_.setTsdfVoxbloxCollisionChecking(
        params_.robot_radius, params_.planning_height, tsdf_layer_);
  } else {
    CHECK_NOTNULL(esdf_layer_);
    problem_setup_.setEsdfVoxbloxCollisionChecking(
        params_.robot_radius, params_.planning_height, esdf_layer_);
  }

  if (!params_.global_params.use_distance_threshold) {
    problem_setup_.setDefaultObjective();
  } else {
    problem_setup_.setLenghtOptimizationObjective(
        params_.global_params.distance_threshold *
        (goal.head<2>() - start.head<2>()).norm());
  }
  if (planner_type_ == kRrtConnect) {
    problem_setup_.setRrtConnect();
  } else if (planner_type_ == kRrtStar) {
    problem_setup_.setRrtStar();
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()
        ->setGoalBias(params_.global_params.goal_bias);
  } else if (planner_type_ == kInformedRrtStar) {
    problem_setup_.setInformedRrtStar();
    problem_setup_.getPlanner()->as<ompl::geometric::InformedRRTstar>()
        ->setGoalBias(params_.global_params.goal_bias);
  } else if (planner_type_ == kPrm) {
    problem_setup_.setPrm();
  } else if (planner_type_ == kBitStar) {
    problem_setup_.setBitStar();
  } else {
    problem_setup_.setDefaultPlanner();
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()
        ->setGoalBias(params_.global_params.goal_bias);
  }

  if (lower_bound_ != upper_bound_) {
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, lower_bound_.x());
    bounds.setLow(1, lower_bound_.y());

    bounds.setHigh(0, upper_bound_.x());
    bounds.setHigh(1, upper_bound_.y());

    // Define start and goal positions.
    problem_setup_.getGeometricComponentStateSpace()
        ->as<ompl::smb::RStateSpace>()
        ->setBounds(bounds);
  }

  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
  double validity_checking_resolution = 0.01;
  if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
    // If bounds are set, set this to approximately one voxel.
    validity_checking_resolution =
        voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
  }
  problem_setup_.setStateValidityCheckingResolution(
      validity_checking_resolution);
}

void VoxbloxOmplRrt::setupTraversabilityProblem(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const grid_map::GridMap &traversability_map) {
  problem_setup_.clear();

  if (optimistic_) {
    CHECK_NOTNULL(tsdf_layer_);
    problem_setup_.setTsdfTraversabilityVoxbloxCollisionChecking(
        params_.robot_radius, params_.planning_height,
        params_.traversability_threshold, params_.maximum_difference_elevation,
        tsdf_layer_, traversability_map);
  } else {
    CHECK_NOTNULL(esdf_layer_);
    ROS_ERROR("[Smb Global Planner] Traversability with Esdf missing at "
              "the moment!");
    problem_setup_.setEsdfVoxbloxCollisionChecking(
        params_.robot_radius, params_.planning_height, esdf_layer_);
  }

  if (!params_.global_params.use_distance_threshold) {
    problem_setup_.setDefaultObjective();
  } else {
    problem_setup_.setLenghtOptimizationObjective(
        params_.global_params.distance_threshold *
        (goal.head<2>() - start.head<2>()).norm());
  }

  if (planner_type_ == kRrtConnect) {
    problem_setup_.setRrtConnect();
  } else if (planner_type_ == kRrtStar) {
    problem_setup_.setRrtStar();
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()
        ->setGoalBias(params_.global_params.goal_bias);
  } else if (planner_type_ == kInformedRrtStar) {
    problem_setup_.setInformedRrtStar();
    problem_setup_.getPlanner()->as<ompl::geometric::InformedRRTstar>()
        ->setGoalBias(params_.global_params.goal_bias);
  } else if (planner_type_ == kPrm) {
    problem_setup_.setPrm();
  } else if (planner_type_ == kBitStar) {
    problem_setup_.setBitStar();
  } else {
    problem_setup_.setDefaultPlanner();
    problem_setup_.getPlanner()->as<ompl::geometric::RRTstar>()
        ->setGoalBias(params_.global_params.goal_bias);
  }

  if (lower_bound_ != upper_bound_) {
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, lower_bound_.x());
    bounds.setLow(1, lower_bound_.y());

    bounds.setHigh(0, upper_bound_.x());
    bounds.setHigh(1, upper_bound_.y());

    // Define start and goal positions.
    problem_setup_.getGeometricComponentStateSpace()
        ->as<ompl::smb::RStateSpace>()
        ->setBounds(bounds);
  }

  // This is a fraction of the space extent! Not actual metric units. For
  // mysterious reasons. Thanks OMPL!
  double validity_checking_resolution = 0.01;
  if ((upper_bound_ - lower_bound_).norm() > 1e-3) {
    // If bounds are set, set this to approximately one voxel.
    validity_checking_resolution =
        voxel_size_ / (upper_bound_ - lower_bound_).norm() / 2.0;
  }
  problem_setup_.setStateValidityCheckingResolution(
      validity_checking_resolution);
}

bool VoxbloxOmplRrt::validStraightLine(const Eigen::Vector3d &start,
                                       const Eigen::Vector3d &goal,
                                       const int n_step,
                                       std::vector<Eigen::Vector3d> &path)
                                       const {
  double delta_x = (goal(0) - start(0)) / static_cast<double>(n_step);
  double delta_y = (goal(1) - start(1)) / static_cast<double>(n_step);
  double delta_z = (goal(2) - start(2)) / static_cast<double>(n_step);

  for (int i = 0; i <= n_step; ++i) {
    Eigen::Vector3d delta_position(delta_x * static_cast<double>(i),
                                   delta_y * static_cast<double>(i),
                                   delta_z * static_cast<double>(i));

    voxblox::Point robot_point =
            (start + delta_position).cast<voxblox::FloatingPoint>();
    voxblox::HierarchicalIndexMap block_voxel_list;
    voxblox::utils::getSphereAroundPoint(
            *tsdf_layer_, robot_point, params_.robot_radius, &block_voxel_list);

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
            block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
              tsdf_layer_->getBlockPtrByIndex(kv.first);

      if (!block_ptr) {
        continue;
      }

      for (const voxblox::VoxelIndex &voxel_index : kv.second) {
        if (!block_ptr->isValidVoxelIndex(voxel_index)) {
          return false;
        }
        const voxblox::TsdfVoxel &tsdf_voxel =
                block_ptr->getVoxelByVoxelIndex(voxel_index);
        if (tsdf_voxel.distance < 0.0f) {
          return false;
        }
      }
    }

    // If no collision is found, then add the position to the storage
    path.push_back(start + delta_position);
  }

  // Add the final position as well - we already know it is valid
  path.push_back(goal);

  return true;
}

bool VoxbloxOmplRrt::validTraversableStraightLine(
        const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
        const int n_step, std::vector<Eigen::Vector3d> &path,
        const grid_map::GridMap &traversability_map) const {

  double delta_x = (goal(0) - start(0)) / static_cast<double>(n_step);
  double delta_y = (goal(1) - start(1)) / static_cast<double>(n_step);
  double delta_z = (goal(2) - start(2)) / static_cast<double>(n_step);

  for (int i = 0; i <= n_step; ++i) {
    Eigen::Vector3d delta_position(delta_x * static_cast<double>(i),
                                   delta_y * static_cast<double>(i),
                                   delta_z * static_cast<double>(i));

    // Check if it is traversable. If not, then return false; if yes,
    // consider position valid; if unknown, check voxblox
    Eigen::Vector3d projected_position;
    TraversabilityStatus traversability_status =
            utility_mapping::getTrasversabilityInformation(
                    traversability_map, (start + delta_position).head(2),
                    params_.planning_height, params_.traversability_threshold,
                    params_.maximum_difference_elevation, projected_position);

    if(traversability_status == TraversabilityStatus::UNTRAVERSABLE) {
      return false;
    } else if(traversability_status == TraversabilityStatus::TRAVERSABLE) {
      path.push_back(projected_position);
      continue;
    } // else: UNKNOWN: check voxblox

    voxblox::Point robot_point =
            projected_position.cast<voxblox::FloatingPoint>();
    voxblox::HierarchicalIndexMap block_voxel_list;
    voxblox::utils::getSphereAroundPoint(
            *tsdf_layer_, robot_point, params_.robot_radius, &block_voxel_list);

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
            block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
              tsdf_layer_->getBlockPtrByIndex(kv.first);

      if (!block_ptr) {
        continue;
      }

      for (const voxblox::VoxelIndex &voxel_index : kv.second) {
        if (!block_ptr->isValidVoxelIndex(voxel_index)) {
          return false;
        }
        const voxblox::TsdfVoxel &tsdf_voxel =
                block_ptr->getVoxelByVoxelIndex(voxel_index);
        if (tsdf_voxel.distance < 0.0f) {
          return false;
        }
      }
    }

    // If no collision is found, then add the position to the storage
    path.push_back(start + delta_position);
  }

  // Add the final position as well - we already know it is valid
  path.push_back(goal);

  return true;
}

bool VoxbloxOmplRrt::getPathBetweenWaypoints(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    std::vector<Eigen::Vector3d> &solution) {
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  if (problem_setup_.solve(params_.global_params.num_seconds_to_plan)) {
    if (problem_setup_.haveExactSolutionPath()) {
      if (params_.global_params.simplify_solution) {
        problem_setup_.reduceVertices();
      }
      if (params_.verbose_planner) {
        problem_setup_.getSolutionPath().printAsMatrix(std::cout);
      }
    } else if (params_.global_params.trust_approx_solution &&
               problem_setup_.haveSolutionPath()) {
      ROS_WARN("[Smb Global Planner] OMPL did not find an exact solution. "
               "Output is an approximate solution! It may be not "
               "feasible for the controller.");
    } else {
      ROS_ERROR("[Smb Global Planner] OMPL could not find an exact "
                "solution.");
      return false;
    }
  }

  if (problem_setup_.haveSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(), solution);
    return true;
  }
  return false;
}

void VoxbloxOmplRrt::setupFromStartAndGoal(const Eigen::Vector3d &start,
                                           const Eigen::Vector3d &goal) {

  if (planner_type_ == kPrm) {
    std::dynamic_pointer_cast<ompl::geometric::PRM>(problem_setup_.getPlanner())
        ->clearQuery();
  } else {
    problem_setup_.clear();
  }

  ompl::base::ScopedState<ompl::smb::RStateSpace> start_ompl(
      problem_setup_.getSpaceInformation());
  ompl::base::ScopedState<ompl::smb::RStateSpace> goal_ompl(
      problem_setup_.getSpaceInformation());

  start_ompl->values[0] = start.x();
  start_ompl->values[1] = start.y();

  goal_ompl->values[0] = goal.x();
  goal_ompl->values[1] = goal.y();

  problem_setup_.setStartAndGoalStates(start_ompl, goal_ompl,
                                       0.5 * voxel_size_);
  problem_setup_.setup();
  /** DELETE (lucaBartolomei)
  if (params_.verbose_planner) {
    problem_setup_.print();
  }
  */

  // Add optimization objective
  if (params_.global_params.use_distance_threshold) {
    problem_setup_.setLenghtOptimizationObjective(
        params_.global_params.distance_threshold *
        (goal.head<2>() - start.head<2>()).norm());
  }
}

void VoxbloxOmplRrt::solutionPathToTrajectoryPoints(
    ompl::geometric::PathGeometric &path,
    std::vector<Eigen::Vector3d> &trajectory_points) const {
  trajectory_points.clear();
  trajectory_points.reserve(path.getStateCount());

  std::vector<ompl::base::State *> &state_vector = path.getStates();

  for (ompl::base::State *state_ptr : state_vector) {
    Eigen::Vector3d mav_position(
        state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[0],
        state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[1],
        params_.planning_height);

    trajectory_points.emplace_back(mav_position);
  }
}

bool VoxbloxOmplRrt::getBestPathTowardGoal(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    std::vector<Eigen::Vector3d> &solution) {
  solution.clear();
  setupFromStartAndGoal(start, goal);

  // Solvin' time!
  bool solution_found = false;
  solution_found =
      problem_setup_.solve(params_.global_params.num_seconds_to_plan);
  if (solution_found) {
    if (problem_setup_.haveSolutionPath()) {
      // Simplify and print.
      if (params_.global_params.simplify_solution) {
        problem_setup_.reduceVertices();
      }
      if (params_.verbose_planner) {
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
  if (planner_data.numStartVertices() < 1) {
    ROS_ERROR("No start vertices in RRT!");
    return false;
  }

  unsigned int min_index = 0;
  double min_distance = std::numeric_limits<double>::max();

  if (planner_data.numVertices() <= 0) {
    ROS_ERROR("No vertices in RRT!");
    return false;
  }

  // Iterate over all vertices. Check which is the closest.
  for (unsigned int i = 0; i < planner_data.numVertices(); i++) {
    const ompl::base::PlannerDataVertex &vertex = planner_data.getVertex(i);
    double distance = getDistanceEigenToState(goal, vertex.getState());

    if (distance < min_distance) {
      min_distance = distance;
      min_index = i;
    }
  }

  unsigned int start_index = planner_data.getStartIndex(0);

  // Get the closest vertex back out, and then get its parents.
  std::vector<Eigen::Vector3d> trajectory_points;

  unsigned int current_index = min_index;
  while (current_index != start_index) {
    // Put this vertex in.
    const ompl::base::PlannerDataVertex &vertex =
        planner_data.getVertex(current_index);

    const ompl::base::State *state_ptr = vertex.getState();
    Eigen::Vector3d mav_position(
        state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[0],
        state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[1],
        state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[2]);

    trajectory_points.emplace_back(mav_position);

    std::vector<unsigned int> edges;
    planner_data.getIncomingEdges(current_index, edges);

    if (edges.empty()) {
      break;
    }

    current_index = edges.front();
  }

  // Finally reverse the vector.
  std::reverse(std::begin(trajectory_points), std::end(trajectory_points));

  solution = trajectory_points;
  return false;
}

double
VoxbloxOmplRrt::getDistanceEigenToState(const Eigen::Vector3d &eigen,
                                        const ompl::base::State *state_ptr) {
  Eigen::Vector3d state_pos(
      state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[0],
      state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[1],
      state_ptr->as<ompl::smb::RStateSpace::StateType>()->values[2]);

  return (eigen - state_pos).norm();
}

} // namespace mav_planning
