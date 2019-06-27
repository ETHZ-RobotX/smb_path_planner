/**
  * @author Luca Bartolomei, V4RL
  * @date   13.06.2019
  */

#include "smb_planner_common/traversability_estimator.h"

namespace smb_planner {

TraversabilityEstimator::TraversabilityEstimator(const ros::NodeHandle &nh)
    : nh_(nh), got_elevation_map_(false) {

  // Read the topics from file
  YAML::Node lconf = YAML::LoadFile(
      ros::package::getPath("smb_planner_common") + "/cfg/topics.yaml");

  // Initialize map / traversability estimator
  traversability_map_ =
      std::make_shared<traversability_estimation::TraversabilityMap>(nh_);

  // Set-up the subscriber
  elevation_map_sub_ = nh_.subscribe(
      lconf["elevationMapMsgName"].as<std::string>(), 1000,
      &smb_planner::TraversabilityEstimator::elevationMapCallback, this);

  // Set-up utilities - use just raw layers
  elevation_map_layers_.push_back("elevation");
  elevation_map_layers_.push_back("upper_bound");
  elevation_map_layers_.push_back("lower_bound");
}

void TraversabilityEstimator::elevationMapCallback(
    const grid_map_msgs::GridMap &msg) {
  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::fromMessage(msg, gridMap);
  got_elevation_map_ = true;

  // Set the traversability
  if (!setupTraversabilityMap(gridMap)) {
    ROS_WARN("Could not set up traversability map!");
  }
}

bool TraversabilityEstimator::setupTraversabilityMap(
    const grid_map::GridMap &gridMap) {

  // Grid map - set layers
  grid_map::GridMap mapWithCheckedLayers = gridMap;
  for (const auto &layer : elevation_map_layers_) {
    if (!mapWithCheckedLayers.exists(layer)) {
      mapWithCheckedLayers.add(layer, 0.0);
    }
  }

  // Store elevation map and compute traversability
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(mapWithCheckedLayers, message);
  traversability_map_->setElevationMap(message);

  if (!traversability_map_->computeTraversability()) {
    ROS_WARN("Traversability Estimation: "
             "cannot compute traversability!");
    return false;
  }
  return true;
}

} // end namespace smb_planner
