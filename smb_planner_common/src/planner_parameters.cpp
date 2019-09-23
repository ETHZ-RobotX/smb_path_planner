/**
  * @author Luca Bartolomei, V4RL
  * @date   13.06.2019
  */

#include "smb_planner_common/planner_parameters.h"

namespace smb_planner {

bool readPlannerParameters(const ros::NodeHandle &nh,
                           PlannerParameters &params) {
  // General parameter for both local and global planners
  if (!nh.getParam("frame_id", params.frame_id)) {
    ROS_WARN("Not specified frame id name. Using 'world'.");
    params.frame_id = "world";
  }

  nh.getParam("input_filepath", params.input_filepath);

  if (!nh.getParam("visualize", params.visualize)) {
    ROS_WARN("Not specified if we should visualize output. "
             "Using 'TRUE'.");
    params.visualize = true;
  }

  if (!nh.getParam("verbose_planner", params.verbose_planner)) {
    ROS_WARN("Not specified if we should verbose_planner output. "
             "Using 'FALSE'.");
    params.verbose_planner = false;
  }

  if (!nh.getParam("voxel_size", params.voxel_size)) {
    ROS_WARN("Not specified voxel_size. Using '0.25 m'.");
    params.voxel_size = 0.25;
  }

  if (!nh.getParam("bounding_box_inflation", params.bounding_box_inflation)) {
    ROS_WARN("Not specified bounding_box_inflation. Using '0.5 m'.");
    params.bounding_box_inflation = 0.5;
  }

  if (!nh.getParam("robot_radius", params.robot_radius)) {
    ROS_WARN("Not specified robot_radius. Using '0.5 m'.");
    params.robot_radius = 0.5;
  }

  if (!nh.getParam("planning_height", params.planning_height)) {
    ROS_WARN("Not specified planning height. Using '0.5 m'.");
    params.planning_height = 0.5;
  }

  if (!nh.getParam("max_initial_rotation", params.max_initial_rotation)) {
    ROS_WARN("Not specified max_initial_rotation. Using '45 deg'.");
    params.max_initial_rotation = M_PI / 4.0;
  } else {
    params.max_initial_rotation *= M_PI / 180.0;
  }

  if (!nh.getParam("threshold_goal_reached", params.threshold_goal_reached)) {
    ROS_WARN("Not specified threshold to consider goal reached. "
             "Using '0.10 m'.");
    params.threshold_goal_reached = 0.10;
  }

  if (!nh.getParam("check_traversability", params.check_traversability)) {
    ROS_WARN("Not specified if planner should check traversability. "
             "Using 'FALSE'.");
    params.check_traversability = false;
  }

  if (!nh.getParam("traversability_threshold",
                   params.traversability_threshold)) {
    ROS_WARN("Not specified if traversability threshold. Using '0.5'.");
    params.traversability_threshold = 0.5;
  }

  if (!nh.getParam("maximum_difference_elevation",
                   params.maximum_difference_elevation)) {
    ROS_WARN("Not specified maximum difference in elevation for "
             "traversability estimation. Using '0.5 m'.");
    params.maximum_difference_elevation = 0.5;
  }

  if (!nh.getParam("n_sensors_traversability",
                   params.n_sensors_traversability)) {
    ROS_WARN("Not specified number of sensors for traversability estimation. "
             "Using '1'.");
    params.n_sensors_traversability = 1;
  }

  params.elevation_maps_weights.resize(params.n_sensors_traversability, 1.0);
  for(int s = 0; s < params.n_sensors_traversability; ++s) {
    if (!nh.getParam("elevation_maps_weights_" + std::to_string(s),
                     params.elevation_maps_weights[s])) {
      ROS_WARN("Weight %d for fused elevation map not specified. Using 1.0", s);
    }
  }

  if (!nh.getParam("v_max", params.v_max)) {
    ROS_WARN("Not specified v_max. Using '0.5 m/s'.");
    params.v_max = 0.5;
  }

  if (!nh.getParam("a_max", params.a_max)) {
    ROS_WARN("Not specified a_max. Using '0.5 m/s2'.");
    params.a_max = 0.5;
  }

  if (!nh.getParam("v_yaw_max", params.v_yaw_max)) {
    ROS_WARN("Not specified v_yaw_max. Using '0.5 rad/s'.");
    params.v_yaw_max = 0.5;
  }

  if (!nh.getParam("a_yaw_max", params.a_yaw_max)) {
    ROS_WARN("Not specified a_yaw_max. Using '0.5 rad/s2'.");
    params.a_yaw_max = 0.5;
  }

  if (!nh.getParam("v_rot_on_spot", params.v_rot_on_spot)) {
    ROS_WARN("Not specified v_rot_on_spot. Using '0.5 rad/s'.");
    params.v_rot_on_spot = 0.5;
  }

  if (!nh.getParam("sampling_dt", params.sampling_dt)) {
    ROS_WARN("Not specified sampling_dt. Using '0.05 s'.");
    params.sampling_dt = 0.05;
  }

  // Read global planner parameters
  if (!readGlobalPlannerParameters(nh, params.global_params)) {
    return false;
  }
  if (!readLocalPlannerParameters(nh, params.local_params)) {
    return false;
  }
  return true;
}

bool readGlobalPlannerParameters(const ros::NodeHandle &nh,
                                 GlobalPlannerParameters &params) {

  if (!nh.getParam("use_global_planner_only", params.use_global_planner_only)) {
    ROS_WARN("Not specified if we should the global planner only. "
             "Using 'FALSE'.");
    params.use_global_planner_only = false;
  }

  if (!nh.getParam("simplify_solution", params.simplify_solution)) {
    ROS_WARN("Not specified if we should simplify solution. "
             "Using 'FALSE'.");
    params.simplify_solution = false;
  }

  if (!nh.getParam("trust_approx_solution", params.trust_approx_solution)) {
    ROS_WARN("Not specified if we should trust approx solution. "
             "Using 'TRUE'.");
    params.trust_approx_solution = true;
  }

  if (!nh.getParam("use_distance_threshold", params.use_distance_threshold)) {
    ROS_WARN("Not specified if we should use_distance_threshold. "
             "Using 'FALSE'.");
    params.use_distance_threshold = false;
  }

  if (!nh.getParam("distance_threshold", params.distance_threshold)) {
    ROS_WARN("Not specified distance_threshold. Using 0.2.");
    params.distance_threshold = 0.2;
  }

  if (!nh.getParam("goal_bias", params.goal_bias)) {
    ROS_WARN("Not specified goal_bias. Using 0.05.");
    params.goal_bias = 0.05;
  }

  if (!nh.getParam("planner_type", params.planner_type)) {
    ROS_WARN("Not specified planner_type. Using 'RRT*'.");
    params.planner_type = 1;
  }

  if (!nh.getParam("num_seconds_to_plan", params.num_seconds_to_plan)) {
    ROS_WARN("Not specified num_seconds_to_plan. Using '2.0 s'.");
    params.num_seconds_to_plan = 2.0;
  }

  if (!nh.getParam("global_timer_dt", params.global_timer_dt)) {
    ROS_WARN("Not specified global_timer_dt. Using '2.0 s'.");
    params.global_timer_dt = 2.0;
  }

  if (!nh.getParam("global_interp_dt", params.global_interp_dt)) {
    ROS_WARN("Not specified global_interp_dt. Using '1.0 s'.");
    params.global_interp_dt = 1.0;
  }

  if (!nh.getParam("optimistic_voxblox", params.optimistic_voxblox)) {
    ROS_WARN("Not specified if planner with voxblox is optimistic. "
             "Using 'TRUE'.");
    params.optimistic_voxblox = true;
  }

  if (!nh.getParam("use_fixed_map_size", params.use_fixed_map_size)) {
    ROS_WARN("Not specified if planner should use fixed map size. "
             "Using 'FALSE'.");
    params.use_fixed_map_size = false;
  }

  if (!nh.getParam("lower_bound_x", params.lower_bound(0))) {
    ROS_ERROR("Not specified lower_bound_x.");
    return false;
  }
  if (!nh.getParam("lower_bound_y", params.lower_bound(1))) {
    ROS_ERROR("Not specified lower_bound_y.");
    return false;
  }
  if (!nh.getParam("lower_bound_z", params.lower_bound(2))) {
    ROS_ERROR("Not specified lower_bound_z.");
    return false;
  }

  if (!nh.getParam("upper_bound_x", params.upper_bound(0))) {
    ROS_ERROR("Not specified upper_bound_x.");
    return false;
  }
  if (!nh.getParam("upper_bound_y", params.upper_bound(1))) {
    ROS_ERROR("Not specified upper_bound_y.");
    return false;
  }
  if (!nh.getParam("upper_bound_z", params.upper_bound(2))) {
    ROS_ERROR("Not specified upper_bound_z.");
    return false;
  }

  return true;
}

bool readLocalPlannerParameters(const ros::NodeHandle &nh,
                                LocalPlannerParameters &params) {

  if (!nh.getParam("local_goal_distance", params.local_goal_distance)) {
    ROS_WARN("Not specified local goal distance. "
             "Using '2.5 m'.");
    params.local_goal_distance = 2.50;
  }

  if (!nh.getParam("sec_ahead_planner", params.sec_ahead_planner)) {
    ROS_WARN("Not specified seconds ahead for planning along "
             "trajectory. Using '0.200 s'.");
    params.sec_ahead_planner = 0.200;
  }

  if (!nh.getParam("local_replan_dt", params.local_replan_dt)) {
    ROS_WARN("Not specified replanning timer frequency for local "
             "planner. Using '0.5 s'.");
    params.local_replan_dt = 0.5;
  }

  if (!nh.getParam("command_dt", params.command_dt)) {
    ROS_WARN("Not specified command timer frequency. "
             "Using '0.5 s'.");
    params.command_dt = 0.5;
  }

  if (!nh.getParam("prediction_horizon_mpc", params.prediction_horizon_mpc)) {
    ROS_WARN("Not specified prediction horizon for MPC. Using '1.0 s'");
    params.prediction_horizon_mpc = 1.0;
  }

  if (!nh.getParam("nq", params.nq)) {
    ROS_WARN("Not specified number of poses for CHOMP. Using '20'.");
    params.nq = 20;
  }

  if (!nh.getParam("dt", params.dt)) {
    ROS_WARN("Not specified time step for CHOMP. Using '0.5 s'.");
    params.dt = 0.5;
  }

  if (!nh.getParam("eta", params.eta)) {
    ROS_WARN("Not specified eta for CHOMP. Using '100'.");
    params.eta = 100.0;
  }

  if (!nh.getParam("lambda", params.lambda)) {
    ROS_WARN("Not specified lambda for CHOMP. Using '1.0'.");
    params.lambda = 1.0;
  }

  if (!nh.getParam("cost_gain", params.cost_gain)) {
    ROS_WARN("Not specified cost gain. Using '10.0'.");
    params.cost_gain = 10.0;
  }

  if (!nh.getParam("eps", params.eps)) {
    ROS_WARN("Not specified eps for obstacle cost function. "
             "Using '2.0 m'.");
    params.eps = 2.0;
  }

  return true;
}

} // end namespace smb_planner
