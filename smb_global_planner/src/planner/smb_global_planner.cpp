/**
 * @author Luca Bartolomei, V4RL
 * @date   13.06.2019
 */

#include "smb_global_planner/planner/smb_global_planner.h"

#include <smb_planner_common/utility_visualization.h>
#include <voxblox/utils/planning_utils.h>

namespace smb_global_planner {

SmbGlobalPlanner::SmbGlobalPlanner(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &nh_private,
                                   const PlannerParameters &params)
    : nh_(nh), nh_private_(nh_private), planning_spinner_(1, &planning_queue_),
      params_(params), has_state_info_(false), perform_planning_(false),
      optimistic_(params.global_params.optimistic_voxblox),
      voxblox_server_(nh_, nh_private_), rrt_(nh_, nh_private_, params_),
      current_state_(Eigen::Vector3d::Zero()), goal_(Eigen::Vector3d::Zero()),
      n_iter_vis_(0) {

  // Initialize Voxblox maps
  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);

  // If we have a previous map, we can load it
  if (!params_.input_filepath.empty()) {
    // Verify that the map has an ESDF layer, otherwise generate it.
    if (!voxblox_server_.loadMap(params_.input_filepath)) {
      ROS_ERROR("[Smb Global Planner] Couldn't load ESDF map!");

      // Check if the TSDF layer is non-empty...
      if (tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) {
        ROS_INFO("[Smb Global Planner] Generating ESDF layer from TSDF.");

        // If so, generate the ESDF layer!
        const bool full_euclidean_distance = true;
        voxblox_server_.updateEsdfBatch(full_euclidean_distance);
      } else {
        ROS_ERROR("[Smb Global Planner] TSDF map also empty! "
                  "Check voxel size!");
      }
    }
  }

  ROS_INFO(
      "[Smb Global Planner] Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // Set up voxblox & related visualization
  voxblox_server_.setTraversabilityRadius(
      static_cast<float>(params_.robot_radius));
  voxblox_server_.setSliceLevel(params_.planning_height);

  // Additional checks for voxblox if we want to add visual output
  if (params_.visualize) {
    voxblox_server_.generateMesh();
    voxblox_server_.setPublishSlices(true);
    voxblox_server_.publishSlices();
    voxblox_server_.publishPointclouds();
    voxblox_server_.publishTraversable();
  }

  // Initialize traversability checker
  if (params_.check_traversability) {
    traversability_estimator_ =
        std::unique_ptr<smb_planner::TraversabilityEstimator>(
            new smb_planner::TraversabilityEstimator(nh_));
  }

  // Set up planner
  rrt_.setRobotRadius(params_.robot_radius);
  rrt_.setOptimistic(optimistic_);
  rrt_.setPlanner(
      VoxbloxOmplRrt::RrtPlannerType(params_.global_params.planner_type));

  rrt_.setTsdfLayer(voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr());
  rrt_.setEsdfLayer(voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr());

  // Initialize ROS services, subscribers and publishers
  initROS();
}

SmbGlobalPlanner::~SmbGlobalPlanner() {
  // Stop timer
  planning_timer_.stop();
}

void SmbGlobalPlanner::initROS() {
  // Read the topics from file
  std::string topic_file("topics");
  nh_.getParam("/smb_global_planner/topic_file", topic_file);
  YAML::Node lconf = YAML::LoadFile(
      ros::package::getPath("smb_planner_common") + "/cfg/" + topic_file +
                            ".yaml");

  // Services
  planner_srv_ = nh_.advertiseService(
      lconf["globalPlanner/plannerServiceName"].as<std::string>(),
      &smb_global_planner::SmbGlobalPlanner::plannerServiceCallback, this);

  // Publishers
  path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      lconf["globalPlanner/visualOutputMsgName"].as<std::string>(), 1, true);
  trajectory_pub_ = nh_.advertise<nav_msgs::Path>(
      lconf["globalPlanner/outputTrajectoryMsgName"].as<std::string>(), 1000);

  // Subscribers
  state_estimation_sub_ = nh_.subscribe(
      lconf["stateEstimatorMsgName"].as<std::string>(), 100,
      &smb_global_planner::SmbGlobalPlanner::stateEstimationCallback, this);
  smb_state_sub_ = nh_.subscribe(
      lconf["smbStateMsgName"].as<std::string>(), 100,
      &smb_global_planner::SmbGlobalPlanner::smbStateCallback, this);

  // Timers
  ros::TimerOptions timer_options(
      ros::Duration(params_.global_params.global_timer_dt),
      boost::bind(&smb_global_planner::SmbGlobalPlanner::plannerTimerCallback,
                  this, _1),
      &planning_queue_);
  planning_timer_ = nh_.createTimer(timer_options);

  // Start the magic
  planning_spinner_.start();
}

void SmbGlobalPlanner::stateEstimationCallback(
    const geometry_msgs::PoseStamped &pose_msg) {
  has_state_info_ = true;
  current_state_(0) = pose_msg.pose.position.x;
  current_state_(1) = pose_msg.pose.position.y;
  current_state_(2) = tf::getYaw(pose_msg.pose.orientation);

  // Check if goal is reached
  checkDistanceGoal();
}

void SmbGlobalPlanner::smbStateCallback(
    const smb_msgs::SmbState &smb_state_msg) {
  has_state_info_ = true;
  current_state_(0) = smb_state_msg.pose.pose.position.x;
  current_state_(1) = smb_state_msg.pose.pose.position.y;
  current_state_(2) = tf::getYaw(smb_state_msg.pose.pose.orientation);

  // Check if goal is reached
  checkDistanceGoal();
}

bool SmbGlobalPlanner::plannerServiceCallback(
    smb_planner_msgs::PlannerService::Request &req,
    smb_planner_msgs::PlannerService::Response &res) {

  if (!has_state_info_) {
    ROS_ERROR("[Smb Global Planner] Planner does not have state info. "
              "Abort planning.");
    return false;
  }

  if (params_.check_traversability) {
    if (!traversability_estimator_->hasElevationMap()) {
      ROS_ERROR("[Smb Global Planner] Planner does not elevation map. "
                "Abort planning.");
      return false;
    }
  }

  // For safety reasons, send stop command to local planner and then plan
  // globally
  sendStopCommand();

  // Else: we can set the goal to the service request value
  goal_ << req.goal_pose.pose.position.x, req.goal_pose.pose.position.y,
      params_.planning_height;

  // Check that we are not in the goal position. If we are, avoid planning!
  if ((goal_.head<2>() - current_state_.head<2>()).norm() <
      2.0 * params_.voxel_size) {
    ROS_WARN("[Smb Global Planner] Robot is already in the goal position! "
             "Abort planning.");
    return false;
  }

  // Set the start pose
  Eigen::Vector3d start(current_state_(0), current_state_(1),
                        params_.planning_height);

  // Visualization
  if (params_.visualize) {
    marker_array_.markers.push_back(createMarkerForState(
        goal_, utility_visualization::Color::Blue(), "goal_position", 0.1));
    marker_array_.markers.push_back(
        createBoundingBoxForState(goal_, "goal_bb"));
    marker_array_.markers.push_back(createMarkerForState(
        start, utility_visualization::Color::Blue(), "start_position", 0.1));
    path_marker_pub_.publish(marker_array_);
  }

  // Setup latest copy of map.
  if (!(esdf_map_ &&
        esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0) &&
      !(tsdf_map_ &&
        tsdf_map_->getTsdfLayerPtr()->getNumberOfAllocatedBlocks() > 0)) {
    ROS_ERROR("[Smb Global Planner] Both maps of voxblox are empty!");
    return false;
  }

  // Check start and goal positions
  // Check that we have observed the goal position
  if (!voxblox_server_.getEsdfMapPtr()->isObserved(goal_)) {
    ROS_WARN("[Smb Global Planner] The goal position has not been "
             "observed yet.");
  }

  // Check if start and goal positions are in free space
  if (!optimistic_) {
    // Here we use the ESDF map and we consider unknown space occupied
    if (getMapDistance(current_state_) < params_.robot_radius) {
      ROS_ERROR_STREAM("[Smb Global Planner] Start ["
                       << current_state_(0) << "," << current_state_(1)
                       << "] is in occupied position.");
      return false;
    } else if (getMapDistance(goal_) < params_.robot_radius) {
      ROS_ERROR_STREAM("[Smb Global Planner] Goal ["
                       << goal_(0) << "," << goal_(1)
                       << "] is in occupied position.");
      return false;
    }
  } else {
    // Here we are optimistic about unknown space and we use the TSDF
    Eigen::Vector3d projection_start(current_state_);
    Eigen::Vector3d projection_goal(goal_);

    TraversabilityStatus traversability_start =
        TraversabilityStatus::TRAVERSABLE;
    TraversabilityStatus traversability_goal =
        TraversabilityStatus::TRAVERSABLE;

    if (params_.check_traversability) {
      traversability_start = utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(),
          current_state_.head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, projection_start);

      traversability_goal = utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(),
          goal_.head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, projection_goal);
    }

    if (traversability_start == TraversabilityStatus::UNTRAVERSABLE ||
        !checkOptimisticMapCollision(projection_start)) {
      ROS_ERROR_STREAM("[Smb Global Planner] Start ["
                       << current_state_(0) << "," << current_state_(1)
                       << "] is in occupied position (optimistic).");
      return false;
    }

    if (traversability_goal == TraversabilityStatus::UNTRAVERSABLE ||
        (voxblox_server_.getEsdfMapPtr()->isObserved(goal_) &&
         !checkOptimisticMapCollision(projection_goal))) {
      ROS_ERROR_STREAM("[Smb Global Planner] Goal ["
                       << goal_(0) << "," << goal_(1)
                       << "] is in occupied position (optimistic).");
      return false;
    }
  }

  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[Smb Global Planner] Set global goal to "
                    << goal_(0) << ", " << goal_(1) << ", " << goal_(2));
  }

  // Update check variable that will fire the planner
  perform_planning_ = true;
  res.success = true;
  return res.success;
}

void SmbGlobalPlanner::plannerTimerCallback(const ros::TimerEvent &event) {

  // If we don't have to perform planning and the current global path is fine
  if (!perform_planning_ && isPathCollisionFree()) {
    return;
  }

  if (params_.verbose_planner) {
    ROS_INFO("[Smb Global Planner] Planning path.");
  }

  // Else: we need to perform planning!
  // For safety reason, we send the stop command to be caught by the local
  // planner
  sendStopCommand();
  perform_planning_ = false;

  // Register start time
  ros::Time start_time = ros::Time::now();

  // Figure out map bounds!
  computeMapBounds(&params_.global_params.lower_bound,
                   &params_.global_params.upper_bound);

  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[Smb Global Planner] Map bounds: "
                    << params_.global_params.lower_bound.transpose() << " to "
                    << params_.global_params.upper_bound.transpose()
                    << " size: "
                    << (params_.global_params.upper_bound -
                        params_.global_params.lower_bound)
                           .transpose());
  }

  // Inflate the bounds a bit.
  Eigen::Vector3d inflation(params_.bounding_box_inflation,
                            params_.bounding_box_inflation, 0.0);
  // Don't in flate in z. ;)
  rrt_.setBounds(params_.global_params.lower_bound - inflation,
                 params_.global_params.upper_bound + inflation);

  // Start state
  Eigen::Vector3d start(current_state_(0), current_state_(1),
                        params_.planning_height);

  // Container where the waypoints of the global path are stored
  std::vector<Eigen::Vector3d> waypoints;

  // Set up problem depending on the desidered configuration
  if (!params_.check_traversability) {
    rrt_.setupProblem(start, goal_);
  } else {
    rrt_.setupTraversabilityProblem(
        start, goal_, traversability_estimator_->getGridMapTraversability());
  }

  if (params_.global_params.use_distance_threshold && params_.verbose_planner) {
    ROS_INFO_STREAM("[Smb Global Planner] Setting cost threshold to "
                    << params_.global_params.distance_threshold *
                           (goal_.head<2>() - start.head<2>()).norm());
  }

  // Planning here!
  bool success = rrt_.getPathBetweenWaypoints(start, goal_, waypoints);
  if (!success) {
    ROS_ERROR_STREAM("[Smb Global Planner] Could not find a path to the "
                     "goal ["
                     << goal_(0) << "," << goal_(1) << "]");
    // Update flag and variables
    perform_planning_ = false;
    waypoints_.clear();

    // Do not send stop command here because we have already sent one when
    // the global planner was triggered
    return;
  }

  double path_length = computePathLength(waypoints);
  int num_vertices = waypoints.size();
  ROS_INFO("[Smb Global Planner] Planner Success? %d Path length: %f "
           "Vertices: %d",
           success, path_length, num_vertices);

  // Interpolate linearly in between waypoints in order to have a global path
  // which is a bit denser. Add the orientation information.
  interpolated_waypoints_.clear();
  waypoints_ = waypoints;

  if (!utility_mapping::rampInterpolatorWaypoints(
          waypoints, interpolated_waypoints_, params_.v_max, params_.a_max,
          params_.global_params.global_interp_dt)) {
    ROS_ERROR("[Smb Global Planner] Could not interpolate global path!");
    return;
  }

  // Update the waypoints after the interpolation and visualize them
  if (params_.visualize) {
    publishVisualization(interpolated_waypoints_);
  }

  if (params_.verbose_planner) {
    ROS_INFO_STREAM("[Smb Global Planner] All timings: "
                    << std::endl
                    << (ros::Time::now() - start_time).toSec() << " s");
    ROS_INFO_STREAM("[Smb Global Planner] Finished planning with start "
                    "point: ["
                    << start(0) << "," << start(1) << "," << start(2)
                    << "] and goal point: [" << goal_(0) << "," << goal_(1)
                    << "," << goal_(2) << "]");
  }

  // Publish trajectory to the MPC
  publishTrajectory();
}

void SmbGlobalPlanner::computeMapBounds(Eigen::Vector3d *lower_bound,
                                        Eigen::Vector3d *upper_bound) const {

  if (esdf_map_ && !params_.global_params.optimistic_voxblox) {
    voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                              lower_bound, upper_bound);
  } else if (tsdf_map_) {
    voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                              lower_bound, upper_bound);
  }
}

double SmbGlobalPlanner::getMapDistance(const Eigen::Vector3d &position) const {

  if (!voxblox_server_.getEsdfMapPtr()) {
    return 0.0;
  }

  double distance = 0.0;
  if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    return 0.0;
  }
  return distance;
}

bool SmbGlobalPlanner::checkOptimisticMapCollision(
    const Eigen::Vector3d &robot_position) {
  // This function returns false if in collision
  voxblox::Layer<voxblox::TsdfVoxel> *layer =
      voxblox_server_.getTsdfMapPtr()->getTsdfLayerPtr();
  voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();
  voxblox::HierarchicalIndexMap block_voxel_list;

  // Extract all the voxels in the sphere for collision checks
  utility_mapping::getSphereAroundPoint(
      *layer, robot_point, params_.robot_radius, &block_voxel_list);

  for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
       block_voxel_list) {
    // Get block -- only already existing blocks are in the list.
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
        layer->getBlockPtrByIndex(kv.first);

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

  // No collision if nothing in the sphere had a negative or 0 distance.
  // Unknown space is unoccupied, since this is a very optimistic global
  // planner.
  return true;
}

double SmbGlobalPlanner::computePathLength(
    const std::vector<Eigen::Vector3d> &waypoints) const {
  double distance = 0.0;
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    Eigen::Vector2d vector(waypoints[i].head<2>() - waypoints[i + 1].head<2>());
    distance += vector.norm();
  }
  return distance;
}

void SmbGlobalPlanner::publishTrajectory() const {

  nav_msgs::Path path_msg;
  path_msg.header.frame_id = params_.frame_id;
  path_msg.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = params_.frame_id;
  pose.header.seq = 0;

  for (auto wp : interpolated_waypoints_) {
    pose.header.stamp = ros::Time(wp(4));
    pose.pose.position.x = wp(0);
    pose.pose.position.y = wp(1);
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(wp(3));

    // Extract the information about elevation if necessary
    if (params_.check_traversability) {
      Eigen::Vector3d projected_position;
      utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(), wp.head<2>(),
          params_.planning_height, params_.traversability_threshold,
          params_.maximum_difference_elevation, projected_position);
      pose.pose.position.z = projected_position(2);
    } else {
      pose.pose.position.z = wp(2);
    }

    path_msg.poses.push_back(pose);
    pose.header.seq++;
  }

  if (trajectory_pub_.getNumSubscribers() > 0) {
    trajectory_pub_.publish(path_msg);
  }
  if (params_.verbose_planner) {
    ROS_INFO("[Smb Global Planner] Published trajectory to the local planner.");
  }
}

void SmbGlobalPlanner::sendStopCommand() const {

  // Send an empty path to the local planner
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = params_.frame_id;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.seq = 0;

  if (trajectory_pub_.getNumSubscribers() > 0) {
    trajectory_pub_.publish(path_msg);
  }
  if (params_.verbose_planner) {
    ROS_WARN("[Smb Global Planner] Published empty trajectory to the local "
             "planner");
  }
}

bool SmbGlobalPlanner::isPathCollisionFree() {
  if (interpolated_waypoints_.empty()) {
    return true;
  }

  // Initialize variables for checks
  bool collision = false;
  TraversabilityStatus traversability_status =
      TraversabilityStatus::TRAVERSABLE;

  // Visualization - show the next path only every n-th iterations
  if (params_.visualize) {
    if (n_iter_vis_ % 10 == 0) {
      publishVisualization(interpolated_waypoints_);
    }
    // Update counter
    n_iter_vis_++;
  }

  // Find initial index from where we should check path
  size_t initial_index = 0;
  Eigen::Vector2d initial_point(interpolated_waypoints_[initial_index](0),
                                interpolated_waypoints_[initial_index](1));
  double dist = (current_state_.head<2>() - initial_point).norm();

  for (size_t i = 1; i < interpolated_waypoints_.size(); ++i) {
    Eigen::Vector2d current_point(interpolated_waypoints_[i](0),
                                  interpolated_waypoints_[i](1));
    double dist_curr = (current_state_.head<2>() - current_point).norm();

    if (dist_curr < dist) {
      dist = dist_curr;
      initial_index = i;
    } else {
      break;
    }
  }

  // Check the path for collisions
  Eigen::Vector3d projection; // this variable will be changed during checks
  for (size_t i = initial_index; i < interpolated_waypoints_.size(); ++i) {
    // Need to set the z value to the planning height. The waypoints in the
    // path are (x, y, theta)
    Eigen::Vector3d check_point(interpolated_waypoints_[i](0),
                                interpolated_waypoints_[i](1),
                                params_.planning_height);

    if (params_.check_traversability) {
      traversability_status = utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(),
          check_point.head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, projection);
      if (traversability_status == TraversabilityStatus::UNTRAVERSABLE) {
        break;
      }
      // If the position is unknown, projection is already set to
      // check_point
    }

    if (!optimistic_) {
      if (getMapDistance(projection) < params_.robot_radius) {
        collision = true;
        break;
      }

      // Check the goal state
      if (!isGoalValid()) {
        return false;
      }
    } else if (!checkOptimisticMapCollision(projection)) {
      collision = true;
      break;
    }
  }

  if (traversability_status == TraversabilityStatus::UNTRAVERSABLE) {

    ROS_ERROR("[Smb Global Planner] Traversability check failed. Forcing "
              "replanning!");

    // Show where the path is not valid
    if (params_.visualize) {
      marker_array_.markers.push_back(
          createMarkerForState(projection, utility_visualization::Color::Blue(),
                               "collision_position", 0.2));
      marker_array_.markers.push_back(
          createBoundingBoxForState(projection, "collision_bb"));
      path_marker_pub_.publish(marker_array_);
    }

    // Send stop command to the local planner
    sendStopCommand();

    perform_planning_ = true;
    return false;

  } else if (collision) {

    ROS_ERROR("[Smb Global Planner] Path is not collision free. Forcing "
              "replanning!");
    // Show where the path is not valid
    if (params_.visualize) {
      marker_array_.markers.push_back(
          createMarkerForState(projection, utility_visualization::Color::Blue(),
                               "collision_position", 0.2));
      marker_array_.markers.push_back(
          createBoundingBoxForState(projection, "collision_bb"));
      path_marker_pub_.publish(marker_array_);
    }

    // Send stop command to the local planner
    sendStopCommand();

    perform_planning_ = true;
    return false;
  }

  return true;
}

bool SmbGlobalPlanner::isGoalValid() {
  // Check if the goal is reacheable
  if (getMapDistance(goal_) < params_.robot_radius) {
    ROS_ERROR_STREAM("[Smb Global Planner] Goal ["
                     << goal_(0) << "," << goal_(1)
                     << "] is in occupied position.");
    perform_planning_ = false;
    return false;
  }
  return true;
}

void SmbGlobalPlanner::checkDistanceGoal() {
  // Check if we have reached the goal
  if (!waypoints_.empty()) {
    if ((goal_.head<2>() - current_state_.head<2>()).norm() <
        params_.threshold_goal_reached) {
      // Clear containers and update flags
      waypoints_.clear();
      perform_planning_ = false;

      // Inform the user and clear visualization markers
      clearAllOldMarkers();
      ROS_INFO("[Smb Global Planner] Goal reached!");
    }
  }
}

visualization_msgs::Marker SmbGlobalPlanner::createMarkerForPath(
    const std::vector<Eigen::Vector3d> &path, const std_msgs::ColorRGBA &color,
    const std::string &name, double scale) const {

  visualization_msgs::Marker path_marker;
  if (params_.check_traversability) {
    return utility_visualization::createMarkerForPathWithTraversability(
        path, color, name, scale, params_.frame_id, params_.planning_height,
        traversability_estimator_->getGridMapTraversability(),
        params_.traversability_threshold, params_.maximum_difference_elevation);
  } else {
    return utility_visualization::createMarkerForPath(
        path, color, name, scale, params_.frame_id, params_.planning_height);
  }
}

visualization_msgs::Marker SmbGlobalPlanner::createMarkerForWaypoints(
    const std::vector<Eigen::Vector3d> &path, const std_msgs::ColorRGBA &color,
    const std::string &name, double scale) {
  if (params_.check_traversability) {
    return utility_visualization::createMarkerForWaypointsWithTraversability(
        path, color, name, scale, params_.frame_id, params_.planning_height,
        traversability_estimator_->getGridMapTraversability(),
        params_.traversability_threshold, params_.maximum_difference_elevation);
  } else {
    return utility_visualization::createMarkerForWaypoints(
        path, color, name, scale, params_.frame_id, params_.planning_height);
  }
}

visualization_msgs::Marker
SmbGlobalPlanner::createMarkerForState(const Eigen::Vector3d &state,
                                       const std_msgs::ColorRGBA &color,
                                       const std::string &name, double scale) {

  if (params_.check_traversability) {
    return utility_visualization::createMarkerForStateWithTraversability(
        state, color, name, scale, params_.frame_id, params_.planning_height,
        traversability_estimator_->getGridMapTraversability(),
        params_.traversability_threshold, params_.maximum_difference_elevation);
  } else {
    return utility_visualization::createMarkerForState(state, color, name,
                                                       scale, params_.frame_id);
  }
}

visualization_msgs::Marker SmbGlobalPlanner::createArrowMarkerForState(
    const Eigen::Vector3d &state, const std_msgs::ColorRGBA &color,
    const std::string &name, double scale) const {

  if (params_.check_traversability) {
    return utility_visualization::createArrowMarkerForStateWithTraversability(
        state, color, name, scale, params_.frame_id, params_.planning_height,
        traversability_estimator_->getGridMapTraversability(),
        params_.traversability_threshold, params_.maximum_difference_elevation);
  } else {
    return utility_visualization::createArrowMarkerForState(
        state, color, name, scale, params_.frame_id, params_.planning_height);
  }
}

visualization_msgs::Marker
SmbGlobalPlanner::createBoundingBoxForState(const Eigen::Vector3d &state,
                                            const std::string &name) const {

  if (params_.check_traversability) {
    return utility_visualization::
        createSphereCollisionForStateWithTraversability(
            state, name, params_.frame_id, params_.robot_radius,
            params_.planning_height,
            traversability_estimator_->getGridMapTraversability(),
            params_.traversability_threshold,
            params_.maximum_difference_elevation);
  } else {
    return utility_visualization::createSphereCollisionForState(
        state, name, params_.frame_id, params_.robot_radius);
  }
}

void SmbGlobalPlanner::clearAllOldMarkers() {
  if (!params_.visualize) {
    return;
  }

  for (size_t i = 0; i < marker_array_.markers.size(); ++i) {
    marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  path_marker_pub_.publish(marker_array_);
  marker_array_.markers.clear();
}

void SmbGlobalPlanner::publishVisualization(
    const std::vector<Eigen::Vector3d> &waypoints) {
  // To make the visualization less messy
  clearAllOldMarkers();

  marker_array_.markers.push_back(createMarkerForPath(
      waypoints, utility_visualization::Color::Green(), "rrt_star", 0.05));
  marker_array_.markers.push_back(createMarkerForWaypoints(
      waypoints, utility_visualization::Color::Red(), "waypoints", 0.10));

  marker_array_.markers.push_back(
      createMarkerForPath(waypoints_, utility_visualization::Color::Orange(),
                          "interpolated_path", 0.05));
  marker_array_.markers.push_back(createMarkerForWaypoints(
      waypoints_, utility_visualization::Color::Orange(),
      "interpolated_waypoints", 0.10));

  const int kPublishEveryNSamples =
      static_cast<int>(std::max(std::floor(waypoints_.size() * 0.05), 1.0));
  for (size_t k = 0; k < waypoints_.size(); ++k) {
    if (k % kPublishEveryNSamples != 0) {
      continue;
    }
    marker_array_.markers.push_back(createArrowMarkerForState(
        waypoints_[k], utility_visualization::Color::Red(),
        "interpolated_yaw" + std::to_string(k), 0.08));
  }
  path_marker_pub_.publish(marker_array_);
}

void SmbGlobalPlanner::publishVisualization(
    const std::vector<Eigen::VectorXd> &waypoints) {

  std::vector<Eigen::Vector3d> interpolated_waypoints;
  for (auto point : waypoints) {
    interpolated_waypoints.push_back(point.head<3>());
  }
  publishVisualization(interpolated_waypoints);
}

void SmbGlobalPlanner::publishVisualization() {
  // To make the visualization less messy
  clearAllOldMarkers();

  // Get closest waypoint to current state
  double distance = (current_state_.head<2>() - waypoints_[0].head<2>()).norm();
  std::vector<Eigen::Vector3d> waypoints_pruned;

  for (auto point : waypoints_) {
    if ((current_state_.head<2>() - point.head<2>()).norm() > distance) {
      waypoints_pruned.push_back(point);
    } else {
      distance = (current_state_.head<2>() - point.head<2>()).norm();
    }
  }

  // Publish the markers
  marker_array_.markers.push_back(createMarkerForPath(
      waypoints_pruned, utility_visualization::Color::Orange(),
      "interpolated_path", 0.05));
  marker_array_.markers.push_back(createMarkerForWaypoints(
      waypoints_pruned, utility_visualization::Color::Orange(),
      "interpolated_waypoints", 0.10));

  const int kPublishEveryNSamples = static_cast<int>(
      std::max(std::floor(waypoints_pruned.size() * 0.05), 1.0));
  for (size_t k = 0; k < waypoints_pruned.size(); ++k) {
    if (k % kPublishEveryNSamples != 0) {
      continue;
    }
    marker_array_.markers.push_back(createArrowMarkerForState(
        waypoints_pruned[k], utility_visualization::Color::Red(),
        "interpolated_yaw" + std::to_string(k), 0.08));
  }
  path_marker_pub_.publish(marker_array_);
}

} // namespace mav_planning
