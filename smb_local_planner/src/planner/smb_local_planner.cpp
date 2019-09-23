/**
 * @author Luca Bartolomei, V4RL
 * @date   14.06.2019
 */

#include "smb_local_planner/smb_local_planner.h"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>

namespace smb_local_planner {

SmbLocalPlanner::SmbLocalPlanner(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private,
                                 const PlannerParameters &params)
    : nh_(nh), nh_private_(nh_private), params_(params),
      voxblox_server_(nh_, nh_private_), planning_spinner_(1, &planning_queue_),
      command_publishing_spinner_(1, &command_publishing_queue_),
      current_state_(Eigen::Vector3d::Zero()), has_state_info_(false),
      start_planning_(false), start_sending_commands_(false), rotate_(false),
      first_rotation_(true), wp_num_(1), path_index_(0),
      iter_initialization_mpc_(0) {

  // Initialize Voxblox maps
  esdf_map_ = voxblox_server_.getEsdfMapPtr();
  CHECK(esdf_map_);
  tsdf_map_ = voxblox_server_.getTsdfMapPtr();
  CHECK(tsdf_map_);

  ROS_INFO(
      "[Smb Local Planner] Size: %f VPS: %lu",
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxel_size(),
      voxblox_server_.getEsdfMapPtr()->getEsdfLayerPtr()->voxels_per_side());

  // Set up voxblox
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
            new smb_planner::TraversabilityEstimator(nh_,
                  params_.elevation_maps_weights));
  }

  // Set up planner
  chomp_solver_ = std::unique_ptr<smb_local_planner::ChompSolver>(
      new smb_local_planner::ChompSolver(esdf_map_, params_));

  // Initialize ROS variables
  initROS();
}

SmbLocalPlanner::~SmbLocalPlanner() {
  // Stop timers
  planning_timer_.stop();
  command_publishing_timer_.stop();

  // Stop spinners
  planning_spinner_.stop();
  command_publishing_spinner_.stop();
}

void SmbLocalPlanner::initROS() {
  // Read the topics from file
  YAML::Node lconf = YAML::LoadFile(
      ros::package::getPath("smb_planner_common") + "/cfg/topics.yaml");

  // Services
  trigger_local_srv_ = nh_.advertiseService(
      lconf["localPlanner/triggerServiceName"].as<std::string>(),
      &smb_local_planner::SmbLocalPlanner::triggerServiceCallback, this);

  // Publishers
  path_marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      lconf["localPlanner/visualOutputMsgName"].as<std::string>(), 1, true);
  trajectory_mpc_pub_ = nh_.advertise<nav_msgs::Path>(
      lconf["localPlanner/outputMPCTrajectoryMsgName"].as<std::string>(), 1000);

  // Subscribers
  state_estimation_sub_ = nh_.subscribe(
      lconf["stateEstimatorMsgName"].as<std::string>(), 100,
      &smb_local_planner::SmbLocalPlanner::stateEstimationCallback, this);
  smb_state_sub_ = nh_.subscribe(
      lconf["smbStateMsgName"].as<std::string>(), 100,
      &smb_local_planner::SmbLocalPlanner::smbStateCallback, this);
  global_planner_sub_ = nh_.subscribe(
      lconf["globalPlanner/outputTrajectoryMsgName"].as<std::string>(), 100,
      &smb_local_planner::SmbLocalPlanner::globalPlannerCallback, this);

  // Timers
  ros::TimerOptions timer_options(
      ros::Duration(params_.local_params.local_replan_dt),
      boost::bind(&smb_local_planner::SmbLocalPlanner::plannerTimerCallback,
                  this, _1),
      &planning_queue_);
  planning_timer_ = nh_.createTimer(timer_options);

  // Start the magic
  planning_spinner_.start();
  command_publishing_spinner_.start();
}

bool SmbLocalPlanner::globalGoalReached(const bool verbose) const {

  // If the global path is empty, it means something is wrong!
  if (global_path_.empty()) {
    return false;
  }

  // Check the last position of the global path --> (X,Y) only!
  Eigen::Vector2d last_waypoint(global_path_.back().head<2>());
  Eigen::Vector2d current_position(current_state_.head<2>());
  if ((last_waypoint - current_position).norm() <=
      params_.threshold_goal_reached) {
    ROS_INFO("[Smb Local Planner] Global goal reached!");
    return true;
  }
  return false;
}

Eigen::Vector3d SmbLocalPlanner::extractLocalGoal() {

  // If the global path is empty, then we can just stay where we are
  if (global_path_.empty()) {
    ROS_WARN("[Smb Local Planner] Global path empty - select current state as"
             " local goal");
    return current_state_;
  }

  // If our previous target is the last waypoint of the global path, then
  // we can just keep using the last waypoint available until we reach it
  if (wp_num_ + 1 >= global_path_.size()) {
    ROS_INFO("[Smb Local Planner] Reached the end of global plan - select "
             "last state as local goal");
    return global_path_.back();
  }

  // Select the closest point of the global path: if we are closer to the
  // next waypoint than to the current one, then we start using the next
  // waypoint as local goal. If the new waypoint is farther away more than
  // a threshold, then we just take as local goal the point on the straight
  // line at the maximum allowed distance.
  double distance_curr_wp =
      (current_state_.head<2>() - global_path_[wp_num_].head<2>()).norm();
  if (distance_curr_wp < params_.voxel_size ||
      distance_curr_wp >
          (current_state_.head<2>() - global_path_[wp_num_ + 1].head<2>())
              .norm()) {
    // Increase the counter for index of waypoints
    wp_num_++;
    double distance =
        (global_path_[wp_num_].head<2>() - current_state_.head<2>()).norm();

    if (distance > params_.local_params.local_goal_distance) {
      Eigen::Vector2d local_goal_pos =
          params_.local_params.local_goal_distance *
              (global_path_[wp_num_].head<2>() - current_state_.head<2>())
                  .normalized() +
          current_state_.head<2>();
      double yaw = utility_mapping::pi2pi(
          current_state_(2) +
          (global_path_[wp_num_](2) - current_state_(2)) *
              params_.local_params.local_goal_distance / distance);
      return Eigen::Vector3d(local_goal_pos(0), local_goal_pos(1), yaw);
    } else {
      return global_path_[wp_num_];
    }

  } else {
    return global_path_[wp_num_];
  }
}

Eigen::Vector3d SmbLocalPlanner::getStartState() const {

  // The start state will coincide with the current pose of the robot in
  // case we don't have a local path (ie we got a new global path or we are
  // starting to plan now) or if we haven't moved yet (ie we have not
  // started sending commands to the controller)
  if (local_path_.empty() || !start_sending_commands_) {
    return current_state_;
  }

  // Assuming we have an interpolated local path (x,y,yaw,t) - get the
  // starting pose by checking where we will be (hopefully) a given number
  // of seconds in the future
  for (int i = 0; i < local_path_.size(); ++i) {
    if (local_path_[i](3) > params_.local_params.sec_ahead_planner) {
      return local_path_[i].head<3>();
    }
  }

  // Else
  return current_state_;
}

void SmbLocalPlanner::interpolateLocalPath() {
  Eigen::VectorXd trajectory(chomp_solver_->getPath().rows() + 3);
  trajectory.block(0, 0, 3, 1) = getStartState();
  trajectory.block(3, 0, chomp_solver_->getPath().rows(), 1) =
      chomp_solver_->getPath();

  // Initialize variables
  double time = 0.0;
  double velocity = 0.0;
  local_path_.clear();

  for (size_t i = 0; i < trajectory.rows() - 3; i += 3) {
    // Extract start and end point for the current segment
    Eigen::Vector2d start(trajectory(i), trajectory(i + 1));
    Eigen::Vector2d end(trajectory(i + 3), trajectory(i + 4));

    // Figure out what the total segment time will be.
    double total_segment_distance = (end - start).norm();
    // Total time needed to get to max speed (or go from max speed to 0).
    double min_acceleration_time = params_.v_max / params_.a_max;
    // The amount of distance covered during the acceleration (or
    // decceleration process).
    double min_acceleration_distance =
        params_.v_max * min_acceleration_time -
        0.5 * params_.a_max * std::pow(min_acceleration_time, 2.0);

    double total_segment_time = 0.0;
    // Case 1: time is shorter than the acceleration and decceleration time.
    if (total_segment_distance < 2 * min_acceleration_distance) {
      total_segment_time =
          2 * std::sqrt(total_segment_distance / params_.a_max);
    } else {
      // Case 2: time is longer than accel + deccel time.
      total_segment_time =
          2 * min_acceleration_time +
          (total_segment_distance - 2 * min_acceleration_distance) /
              params_.v_max;
    }
    size_t num_elements = total_segment_time / params_.sampling_dt;
    Eigen::Vector2d path_direction = (end - start).normalized();

    // Treat this as a 1D problem since it is. ;)
    double position = 0.0;

    // Separate the time between total time and local segment time
    double current_time_trajectory = time;
    int64_t current_time_segment = 0.0;

    for (size_t j = 0; j < num_elements; ++j) {
      // Integrate velocity to get position.
      position += velocity * params_.sampling_dt;

      // Figure out if we're accelerating, deccelerating, or neither.
      // Handle Case 1 first:
      if (total_segment_time < min_acceleration_time * 2) {
        if (current_time_segment < total_segment_time / 2.0) {
          velocity += params_.a_max * params_.sampling_dt;
        } else {
          velocity -= params_.a_max * params_.sampling_dt;
        }
      } else {
        // Case 2
        if (position <= min_acceleration_distance) {
          velocity += params_.a_max * params_.sampling_dt;
        } else if ((total_segment_distance - position) <=
                   min_acceleration_distance) {
          velocity -= params_.a_max * params_.sampling_dt;
        }
      }

      // Make sure to meet constraints (could be passed/missed due to
      // discretization error).
      if (position > total_segment_distance) {
        position = total_segment_distance;
      }
      if (velocity > params_.v_max) {
        velocity = params_.v_max;
      }
      if (velocity < 0) {
        velocity = 0;
      }

      // Save the interpolated path
      local_path_.push_back(
          Eigen::Vector4d(start(0) + path_direction(0) * position,
                          start(1) + path_direction(1) * position,
                          trajectory(i + 2), current_time_trajectory));

      // Update times
      current_time_trajectory += params_.sampling_dt;
      current_time_segment += params_.sampling_dt;
    }
    time = current_time_trajectory + params_.sampling_dt;
  }

  // Add end point
  local_path_.push_back(Eigen::Vector4d(
      trajectory(trajectory.rows() - 3), trajectory(trajectory.rows() - 2),
      trajectory(trajectory.rows() - 1), time));
}

bool SmbLocalPlanner::isPathCollisionFree(
    const std::vector<Eigen::Vector4d> &path) const {

  for (const Eigen::Vector4d &point : path) {

    Eigen::Vector3d position(point(0), point(1), params_.planning_height);
    if (params_.check_traversability) {
      TraversabilityStatus traversability_status =
          utility_mapping::getTrasversabilityInformation(
              traversability_estimator_->getGridMapTraversability(),
              point.head<2>(), params_.planning_height,
              params_.traversability_threshold,
              params_.maximum_difference_elevation, position);
      if (traversability_status == TraversabilityStatus::UNTRAVERSABLE) {
        ROS_ERROR("[Smb Local Planner] Navigating in untraversable "
                  "space!");
        return false;
      }
    }
    if (getMapDistance(position) < params_.robot_radius - 0.1) {
      ROS_ERROR("[Smb Local Planner] Robot path will bring to "
                "collision!");
      return false;
    }
  }
  return true;
}

double SmbLocalPlanner::getMapDistance(const Eigen::Vector3d &position) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_map_->getDistanceAtPosition(position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double
SmbLocalPlanner::getMapDistanceAndGradient(const Eigen::Vector3d &position,
                                           Eigen::Vector3d *gradient) const {
  double distance = 0.0;
  const bool kInterpolate = false;
  if (!esdf_map_->getDistanceAndGradientAtPosition(position, kInterpolate,
                                                   &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

bool SmbLocalPlanner::findIntermediateGoal(const Eigen::Vector2d &start_point,
                                           const Eigen::Vector2d &goal_point,
                                           double step_size,
                                           Eigen::Vector2d *goal_out) const {

  Eigen::Vector2d new_goal = goal_point;

  Eigen::Vector2d distance_to_waypoint =
      goal_point.head<2>() - start_point.head<2>();
  double planning_distance = distance_to_waypoint.norm();
  if (planning_distance > params_.local_params.local_goal_distance)
    planning_distance = params_.local_params.local_goal_distance;

  Eigen::Vector2d direction_to_waypoint = distance_to_waypoint.normalized();
  bool success = false;
  Eigen::Vector2d new_goal_temp;

  while (planning_distance >= 0.0) {
    success = getNearestFreeSpaceToPoint(new_goal.head<2>(), step_size,
                                         &new_goal_temp);
    new_goal = new_goal_temp;
    if (success) {
      break;
    }
    planning_distance -= step_size;
    new_goal.head<2>() =
        start_point.head<2>() + planning_distance * direction_to_waypoint;
  }

  if (success) {
    *goal_out = new_goal;
  }
  return success;
}

bool SmbLocalPlanner::getNearestFreeSpaceToPoint(
    const Eigen::Vector2d &pos, double step_size,
    Eigen::Vector2d *new_pos) const {

  CHECK(esdf_map_);
  Eigen::Vector3d final_pos(pos(0), pos(1), params_.planning_height);
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

  const size_t kMaxIter = 20;
  for (size_t i = 0; i < kMaxIter; i++) {
    double distance = getMapDistanceAndGradient(final_pos, &gradient);
    if (distance >= params_.robot_radius) {
      *new_pos = final_pos.head<2>();
      return true;
    }

    if (gradient.norm() > 1e-6) {
      final_pos += gradient.normalized() * step_size;
    }
  }
  return false;
}

void SmbLocalPlanner::sendStopCommand() {

  // Publish stop command to the MPC
  nav_msgs::Path stop_path;
  stop_path.header.frame_id = params_.frame_id;
  stop_path.header.seq = 0;
  stop_path.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = params_.frame_id;
  pose.header.stamp = ros::Time(0.0);
  pose.header.seq = 0;

  pose.pose.position.x = current_state_(0);
  pose.pose.position.y = current_state_(1);
  pose.pose.position.z = params_.planning_height;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_state_(2));
  stop_path.poses.push_back(pose);

  // Hack: to stop the robot send the current position (need to have a
  // "trajectory" that is long enough in time
  double time = params_.sampling_dt;
  while(time <= params_.local_params.prediction_horizon_mpc) {
    pose.header.stamp = ros::Time(time);
    pose.header.seq++;
    stop_path.poses.push_back(pose);
    time += params_.sampling_dt;
  }

  trajectory_mpc_pub_.publish(stop_path);

  // Clear the command buffer as well, otherwise the robot will keep moving!
  resetLocalPlanner();

  // Delete all old markers
  deleteAllMarkers();

  if(params_.verbose_planner) {
    ROS_WARN("[Smb Local Planner] Published empty trajectory to the MPC");
  }
}

void SmbLocalPlanner::sendRotationCommand() const {

  // Standard ROS message
  nav_msgs::Path path_rotate;
  double time = 0.0;

  path_rotate.header.frame_id = params_.frame_id;
  path_rotate.header.stamp = ros::Time::now();
  path_rotate.header.seq = 0;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = params_.frame_id;
  pose.header.seq = 0;

  for (std::vector<Eigen::Vector4d>::const_iterator it =
           commands_rotation_.begin();
       it < commands_rotation_.end(); ++it) {

    pose.header.stamp = ros::Time(time);
    pose.pose.position.x = (*it)(0);
    pose.pose.position.y = (*it)(1);

    if (params_.check_traversability) {
      Eigen::Vector3d projected_position;
      utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(),
          (*it).head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, projected_position);
      pose.pose.position.z = projected_position(2);
    } else {
      pose.pose.position.z = params_.planning_height;
    }

    pose.pose.orientation = tf::createQuaternionMsgFromYaw((*it)(2));

    path_rotate.poses.push_back(pose);
    pose.header.seq++;

    // Times
    time += params_.sampling_dt;
  }

  if (!path_rotate.poses.empty() &&
      trajectory_mpc_pub_.getNumSubscribers() > 0) {
    trajectory_mpc_pub_.publish(path_rotate);
  }
}

bool SmbLocalPlanner::interpolateRotation() {
  // Time and yaw
  double time = 0.0;
  double yaw = current_state_(2);

  Eigen::Vector2d direction_rot =
      (global_path_[1].head<2>() - current_state_.head<2>()).normalized();
  double yaw_path = std::atan2(direction_rot(1), direction_rot(0));

  // Add current position
  commands_rotation_.push_back(
      Eigen::Vector4d(current_state_(0), current_state_(1), yaw, time));

  // Check if the difference in angle exceeds the limit
  if (std::fabs(yaw_path - current_state_(2)) >= params_.max_initial_rotation) {

    // Number of steps
    double delta = yaw_path - current_state_(2);
    if (delta > M_PI)
      delta -= 2.0 * M_PI;
    if (delta < -M_PI)
      delta += 2.0 * M_PI;
    size_t num_elements =
        std::fabs(delta) / (params_.v_rot_on_spot * params_.sampling_dt);

    for (size_t i = 0; i < num_elements; ++i) {
      time += params_.sampling_dt;
      yaw += delta / num_elements;
      if (yaw > 2.0 * M_PI)
        yaw -= 2.0 * M_PI;
      if (yaw < -2.0 * M_PI)
        yaw += 2.0 * M_PI;
      commands_rotation_.push_back(
          Eigen::Vector4d(current_state_(0), current_state_(1), yaw, time));
    }

    // Check that the trajectory we are going to send is longer than the MPC
    // prediction time horizon
    while(time <= params_.local_params.prediction_horizon_mpc) {
      time += params_.sampling_dt;

      // This is a bad hack. Basically we are forcing the robot to stay in
      // the final state for long enough
      commands_rotation_.push_back(
              Eigen::Vector4d(current_state_(0), current_state_(1), yaw, time));
    }

    // Print statistics to user
    if (num_elements > 0) {
      if(params_.verbose_planner) {
        ROS_INFO("[Smb Local Planner] Add initial rotation with %zd points",
                 num_elements);
      }
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool SmbLocalPlanner::hasValidCommands() {
  std::lock_guard<std::recursive_mutex> guard(commands_mutex_);

  // return true if the commands buffer is not empty
  return !commands_.empty();
}

void SmbLocalPlanner::resetLocalPlanner() {

  std::lock_guard<std::recursive_mutex> guard(commands_mutex_);

  // Clear containers
  local_path_.clear();
  commands_.clear();
  commands_rotation_.clear();

  // Reinitialize indices
  wp_num_ = 1;
  path_index_ = 0;
  iter_initialization_mpc_ = 0;
  first_rotation_ = true;
  rotate_ = false;
}

bool SmbLocalPlanner::startSendingCommands() {
  if (!global_path_.empty()) {
    if(params_.verbose_planner) {
      ROS_INFO("[Smb Local Planner] Starting sending commands.");
    }
    start_sending_commands_ = true;

    // Need advanced timer options to assign callback queue to this timer.
    ros::TimerOptions timer_options_command(
        ros::Duration(params_.local_params.command_dt),
        boost::bind(
            &smb_local_planner::SmbLocalPlanner::commandPublishTimerCallback,
            this, _1),
        &command_publishing_queue_);
    command_publishing_timer_ = nh_.createTimer(timer_options_command);
    return true;

  } else {
    ROS_ERROR("[Smb Local Planner] Do not have a global path. Cannot "
              "send commands!");
    start_sending_commands_ = false;
    return false;
  }
}

bool SmbLocalPlanner::triggerServiceCallback(std_srvs::Empty::Request &req,
                                             std_srvs::Empty::Response &res) {

  if (!global_path_.empty()) {
    ROS_INFO("[Smb Local Planner] Triggered local planner.");
    startSendingCommands();

    // Set useful flags
    start_planning_ = true;
    first_rotation_ = true;
    return true;
  } else {
    ROS_ERROR("[Smb Local Planner] Could not trigger local planner - "
              "global path is empty!");
    start_planning_ = false;
    return false;
  }
}

void SmbLocalPlanner::stateEstimationCallback(
    const geometry_msgs::PoseStamped &pose_msg) {
  has_state_info_ = true;
  current_state_(0) = pose_msg.pose.position.x;
  current_state_(1) = pose_msg.pose.position.y;
  current_state_(2) = tf::getYaw(pose_msg.pose.orientation);
}

void SmbLocalPlanner::smbStateCallback(
        const smb_msgs::SmbState &smb_state_msg) {
  has_state_info_ = true;
  current_state_(0) = smb_state_msg.pose.pose.position.x;
  current_state_(1) = smb_state_msg.pose.pose.position.y;
  current_state_(2) = tf::getYaw(smb_state_msg.pose.pose.orientation);
}

void SmbLocalPlanner::globalPlannerCallback(
    const nav_msgs::PathConstPtr &path_msg) {

  // Reset the parameters, because we think we start planning from scratch
  // every time we receive a global path
  resetLocalPlanner();

  // Check if the path message is empty - if so, send the stop command to MPC
  if (path_msg->poses.empty()) {
    if(params_.verbose_planner) {
      ROS_INFO("[Smb Local Planner] Received stop command from global "
               "planner!");
    }
    sendStopCommand();
    return;
  }

  // Store the global path in a vector of eigen
  if(params_.verbose_planner) {
    ROS_INFO("[Smb Local Planner] Received new global path!");
  }

  global_path_.clear();
  for (int i = 0; i < path_msg->poses.size(); ++i) {
    global_path_.push_back(Eigen::Vector3d(
        path_msg->poses[i].pose.position.x, path_msg->poses[i].pose.position.y,
        tf::getYaw(path_msg->poses[i].pose.orientation)));
  }

  // Compute the initial yaw rotation to be performed to face in the
  // direction of the first waypoint of the global path
  Eigen::Vector2d direction_rot =
      (global_path_[wp_num_].head<2>() - current_state_.head<2>()).normalized();
  initial_yaw_rot_ =
      utility_mapping::pi2pi(std::atan2(direction_rot(1), direction_rot(0)));

  // In this case, where the global path is valid, we stop to start
  // following the new global path
  if (start_sending_commands_) {
    sendStopCommand();
  }
}

void SmbLocalPlanner::plannerTimerCallback(const ros::TimerEvent &event) {

  // We will perform planning only if we have a global path, the global
  // goal has not been reached yet
  if ((!start_planning_ && !globalGoalReached(true)) || global_path_.empty()) {
    return;
  }

  if (!has_state_info_) {
    ROS_WARN("[Smb Local Planner] Do not have state info - cannot plan!");
    return;
  }

  // Check if we need to rotate on spot - just send the command once for
  // the first waypoint
  if (wp_num_ == 1) {
    if (std::fabs(initial_yaw_rot_ - current_state_(2)) >
        params_.max_initial_rotation) {

      // This check is required to send the command for rotation on
      // spot only once
      if (!rotate_) {
        interpolateRotation();
        rotate_ = true;
      }
      return;
    } else {
      rotate_ = false;
    }
  }

  // Else : we perform local planning from the current state to the goal on
  // the global path

  // Check if chunk is collision free - if it is, do not replan
  // otherwise replan from starting point to end point
  Eigen::Vector3d local_goal(extractLocalGoal());
  Eigen::Vector3d start_state(getStartState());

  // If we have to rotate on spot, then wait planning
  if (hasValidCommands()) {
    size_t replan_start_index;
    Eigen::Vector2d new_goal_position;

    {
      std::lock_guard<std::recursive_mutex> guard(commands_mutex_);
      replan_start_index =
          std::min(path_index_ + static_cast<size_t>(
                                     (params_.local_params.sec_ahead_planner) /
                                     params_.sampling_dt),
                   commands_.size());

      std::vector<Eigen::Vector4d> path_chunk;
      std::copy(commands_.begin() + replan_start_index, commands_.end(),
                std::back_inserter(path_chunk));
      if (path_chunk.size() == 0) {
        path_chunk.push_back(commands_.back());
      }

      bool path_chunk_collision_free = isPathCollisionFree(path_chunk);
      if(params_.verbose_planner) {
        ROS_INFO("[Smb Local Planner] Existing chunk is collision free? %d",
                 path_chunk_collision_free);
      }

      if ((path_chunk.back().head<2>() - local_goal.head<2>()).norm() <
          params_.threshold_goal_reached) {
        // Collision check the remaining chunk of the trajectory.
        if (path_chunk_collision_free) {
          if(params_.verbose_planner) {
            ROS_INFO("[Smb Local Planner] Current plan is valid, just "
                     "rollin' with it.");
          }
          wp_num_++;
          return;
        }
      }

      // Get the new local path replanning the chunk
      // Select local goal first
      const double kglobalGoalReachedRange = esdf_map_->voxel_size();
      if ((local_goal.head<2>() - start_state.head<2>()).norm() <
          kglobalGoalReachedRange) {
        ROS_INFO("[Smb Local Planner] Local goal already reached!");
        return;
      }

      Eigen::Vector2d distance_to_waypoint =
          local_goal.head<2>() - start_state.head<2>();
      double planning_distance =
          (local_goal.head<2>() - start_state.head<2>()).norm();
      Eigen::Vector2d direction_to_waypoint = distance_to_waypoint.normalized();

      if (planning_distance > params_.local_params.local_goal_distance) {
        local_goal.head<2>() =
            start_state.head<2>() +
            params_.local_params.local_goal_distance * direction_to_waypoint;
        planning_distance = params_.local_params.local_goal_distance;
      }

      bool goal_found = true;
      new_goal_position = local_goal.head<2>();
      if (getMapDistance(Eigen::Vector3d(local_goal(0), local_goal(1),
                                         params_.planning_height)) <
          params_.robot_radius) {
        const double step_size = esdf_map_->voxel_size();
        goal_found =
            findIntermediateGoal(start_state.head<2>(), local_goal.head<2>(),
                                 step_size, &new_goal_position);
      }

      if (!goal_found) {
        return;
      }
    }

    local_goal.head<2>() = new_goal_position;
    bool success(false);
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    if (params_.check_traversability) {
      success = chomp_solver_->solve(
          start_state, local_goal,
          traversability_estimator_->getGridMapTraversability());
    } else {
      success = chomp_solver_->solve(start_state, local_goal);
    }

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;

    if (success) {
      if(params_.verbose_planner) {
        ROS_INFO("[Smb Local Planner] Found local path in %1.5f s",
                 elapsed_seconds.count());
      }
    } else {
      if(params_.verbose_planner) {
        ROS_ERROR("[Smb Local Planner] Could not find local path in %1.5f s",
                  elapsed_seconds.count());
      }
      return;
    }

    // Remove what was in the trajectory before.
    std::lock_guard<std::recursive_mutex> guard(commands_mutex_);
    if (replan_start_index < commands_.size()) {
      commands_.erase(commands_.begin() + replan_start_index, commands_.end());
    }
    // Stick the new one in.
    interpolateLocalPath();
    commands_.insert(commands_.end(), local_path_.begin(), local_path_.end());

  } else {

    // In this case the command buffer is empty and we start to plan from
    // scratch
    if (chomp_solver_->solve(start_state, local_goal)) {
      if(params_.verbose_planner) {
        ROS_INFO("[Smb Local Planner] Found local path");
      }
    } else {
      if(params_.verbose_planner) {
        ROS_ERROR("[Smb Local Planner] Could not find local path");
      }
      local_path_.clear();
      return;
    }

    // Interpolate local path and add it to the command buffer
    interpolateLocalPath();
    std::lock_guard<std::recursive_mutex> guard(commands_mutex_);
    for (int i = 0; i < local_path_.size(); ++i) {
      commands_.push_back(local_path_[i]);
      if (local_path_[i](3) >= params_.local_params.sec_ahead_planner) {
        break;
      }
    }
  }

  // Visualize local path and interpolate
  publishVisualization();
}

void SmbLocalPlanner::commandPublishTimerCallback(
    const ros::TimerEvent &event) {

  if (!start_sending_commands_) {
    return;
  }

  if (iter_initialization_mpc_ == 0) {
    if(params_.verbose_planner) {
      ROS_INFO("[Smb Local Planner] Initializing communication with MPC");
    }
    sendStopCommand();
    iter_initialization_mpc_++;
  }

  if (iter_initialization_mpc_ < int(1.0 / params_.local_params.command_dt)) {
    iter_initialization_mpc_++;
    return;
  }

  std::lock_guard<std::recursive_mutex> guard(commands_mutex_);
  if (commands_.empty() && commands_rotation_.empty()) {
    if(params_.verbose_planner) {
      ROS_WARN("[Smb Local Planner] Command buffer empty");
    }
    return;
  }

  // Check if we need to do rotation on spot
  if (rotate_ && first_rotation_) {
    if(params_.verbose_planner) {
      ROS_INFO("[Smb Local Planner] Staring to send commands for rotation "
               "on spot!");
    }
    first_rotation_ = false;
    sendRotationCommand();
    return;
  }

  if(globalGoalReached()) {
    return;
  }

  // Check how many poses in the buffer we have to publish to the controller
  constexpr size_t kQueueBuffer = 0;
  size_t number_to_publish = std::min<size_t>(
      std::floor(params_.local_params.command_dt / params_.sampling_dt),
      commands_.size() - path_index_);

  size_t starting_index = 0;
  if (path_index_ != 0) {
    starting_index = path_index_ + kQueueBuffer;
    if (starting_index >= commands_.size()) {
      starting_index = path_index_;
    }
  }

  int mpc_prediction_horizon = static_cast<int>(std::floor(
      params_.local_params.prediction_horizon_mpc / params_.sampling_dt));
  size_t number_to_publish_with_buffer = std::min<size_t>(
      number_to_publish + mpc_prediction_horizon - kQueueBuffer,
      commands_.size() - starting_index);

  // Final limits of the buffer to be published
  auto first_sample = commands_.begin() + starting_index;
  auto last_sample = first_sample + number_to_publish_with_buffer;

  // Initialize the messages to publish
  nav_msgs::Path path_mpc;
  path_mpc.header.frame_id = params_.frame_id;
  path_mpc.header.stamp = ros::Time::now();
  path_mpc.header.seq = 0;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = params_.frame_id;
  pose.header.seq = 0;

  // Time for stamping trajectory
  double time = 0.0;

  for (std::vector<Eigen::Vector4d>::iterator it = first_sample;
       it < last_sample; ++it) {
    // Pose message
    pose.header.stamp = ros::Time(time);
    pose.pose.position.x = (*it)(0);
    pose.pose.position.y = (*it)(1);

    Eigen::Vector3d projected_position;
    if (params_.check_traversability) {

      utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(),
          (*it).head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, projected_position);
      pose.pose.position.z = projected_position(2);
    } else {
      pose.pose.position.z = params_.planning_height;
    }

    pose.pose.orientation = tf::createQuaternionMsgFromYaw((*it)(2));
    path_mpc.poses.push_back(pose);

    // Times
    time += params_.sampling_dt;
    pose.header.seq++;
  }

  // Update the starting index to publish commands
  path_index_ += number_to_publish;

  if (trajectory_mpc_pub_.getNumSubscribers() > 0 && !path_mpc.poses.empty()) {
    if(params_.verbose_planner) {
      ROS_INFO("[Smb Local Planner] Published trajectory with %zd points",
               path_mpc.poses.size());
    }
    trajectory_mpc_pub_.publish(path_mpc);
  }
}

void SmbLocalPlanner::publishVisualization() {
  marker_array_.markers.clear();
  marker_array_.markers.push_back(createLocalPathLineStripMarkers());

  // Uncomment this line to see the interpolated path as sphere markers
  // marker_array.markers.push_back(createWaypointsMarkers());

  if (path_marker_pub_.getNumSubscribers() > 0) {
    path_marker_pub_.publish(marker_array_);
  }
}

void SmbLocalPlanner::deleteAllMarkers() {
    for(int i=0; i<marker_array_.markers.size(); ++i) {
        marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    if (path_marker_pub_.getNumSubscribers() > 0) {
        path_marker_pub_.publish(marker_array_);
    }
}

visualization_msgs::Marker SmbLocalPlanner::createWaypointsMarkers() const {
  // Get the path from chomp and the publish as a marker array
  Eigen::VectorXd path = chomp_solver_->getPath();
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = "chomp_solution";
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.lifetime = ros::Duration(0);
  marker.id = 0;
  marker.color = utility_visualization::Color::Red();

  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  for (int i = 0; i < path.size(); i += chomp_solver_->getSpaceDimension()) {
    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(
        Eigen::Vector3d(path(i), path(i + 1), params_.planning_height),
        point_msg);
    marker.points.push_back(point_msg);
  }
  return marker;
}

visualization_msgs::Marker
SmbLocalPlanner::createLocalPathLineStripMarkers() const {
  // Get the path from chomp and the publish as a marker array
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = "local_path";
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration(0);
  marker.color = utility_visualization::Color::Teal();
  marker.id = 0;
  marker.scale.x = 0.05;

  for (int i = 0; i < commands_.size(); i++) {
    geometry_msgs::Point point_msg;

    if (params_.check_traversability) {
      Eigen::Vector3d position;
      utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(),
          commands_[i].head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, position);
      tf::pointEigenToMsg(position, point_msg);
    } else {
      tf::pointEigenToMsg(Eigen::Vector3d(commands_[i](0), commands_[i](1),
                                          params_.planning_height),
                          point_msg);
    }
    marker.points.push_back(point_msg);
  }
  return marker;
}

visualization_msgs::Marker SmbLocalPlanner::createPathMarkers() const {
  // Get the path from chomp and the publish as a marker array
  visualization_msgs::Marker marker;
  marker.header.frame_id = params_.frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = "waypoints_local_path";
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.lifetime = ros::Duration(0);
  marker.color = utility_visualization::Color::Orange();
  marker.id = 0;

  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  for (int i = 0; i < local_path_.size(); i++) {
    geometry_msgs::Point point_msg;
    if (params_.check_traversability) {
      Eigen::Vector3d projected_position;
      utility_mapping::getTrasversabilityInformation(
          traversability_estimator_->getGridMapTraversability(),
          local_path_[i].head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, projected_position);
      tf::pointEigenToMsg(projected_position, point_msg);
    } else {
      tf::pointEigenToMsg(Eigen::Vector3d(local_path_[i](0), local_path_[i](1),
                                          params_.planning_height),
                          point_msg);
    }
    marker.points.push_back(point_msg);
  }
  return marker;
}

} // end namespace smb_local_planner
