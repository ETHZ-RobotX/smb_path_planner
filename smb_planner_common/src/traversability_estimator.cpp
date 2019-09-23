/**
  * @author Luca Bartolomei, V4RL
  * @date   13.06.2019
  */

#include "smb_planner_common/traversability_estimator.h"

#include <numeric>

namespace smb_planner {

TraversabilityEstimator::TraversabilityEstimator(const ros::NodeHandle &nh,
     const std::vector<double> &maps_weights, const int n_sensors)
    : nh_(nh), n_sensors_(n_sensors), weights_(maps_weights),
      weights_sum_(0.0) {

  // Read the topics from file
  YAML::Node lconf = YAML::LoadFile(
      ros::package::getPath("smb_planner_common") + "/cfg/topics.yaml");

  // Initialize map / traversability estimator
  traversability_map_ =
      std::make_shared<traversability_estimation::TraversabilityMap>(nh_);

  // Set-up the subscriber
  if(n_sensors == 1) {
    elevation_map_subs_.resize(1);
    elevation_map_subs_[0] = nh_.subscribe(
        lconf["elevationMapMsgName"].as<std::string>(), 1000,
        &smb_planner::TraversabilityEstimator::elevationMapCallback, this);
  } else {
    elevation_grid_maps_.resize(n_sensors_);
    elevation_map_subs_.reserve(n_sensors_);
    for(int i = 0; i < n_sensors_; ++i) {
      std::string topic_sensor_i(
        "/elevation_mapping_" + std::to_string(i) + "/elevation_map");
      ros::Subscriber elevation_map_sensor_sub = nh_.subscribe<
              grid_map_msgs::GridMap>(
                      topic_sensor_i, 1000,
                      boost::bind(
                          &smb_planner::TraversabilityEstimator::
                             elevationSensorMapCallback,
                          this, _1, i));
      elevation_map_subs_.push_back(elevation_map_sensor_sub);
    }
  }      

  // Set-up utilities - use just raw layers
  got_elevation_map_.resize(n_sensors_, false);

  elevation_map_layers_.push_back("elevation");
  elevation_map_layers_.push_back("upper_bound");
  elevation_map_layers_.push_back("lower_bound");

  // Get the weights values from the ROS server
  if(n_sensors_ == 1) {
    weights_[0] = 1.0;
  }
  for(int s = 0; s < n_sensors_; ++s) {
    weights_sum_ += weights_[s];
  }
}

void TraversabilityEstimator::elevationMapCallback(
    const grid_map_msgs::GridMap &msg) {
  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::fromMessage(msg, gridMap);
  got_elevation_map_[0] = true;

  // Set the traversability
  if (!setupTraversabilityMap(gridMap)) {
    ROS_WARN("Could not set up traversability map!");
  }
}

void TraversabilityEstimator::elevationSensorMapCallback(
    const grid_map_msgs::GridMapConstPtr &msg, const int n_sensor) {
  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::fromMessage(*msg, gridMap);
  got_elevation_map_[n_sensor] = true;
  elevation_grid_maps_[n_sensor] = gridMap;

  // Set the traversability
  if (!setupFusedTraversabilityMap()) {
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
    ROS_WARN("Traversability Estimation: cannot compute traversability!");
    return false;
  }
  return true;
}

bool TraversabilityEstimator::setupFusedTraversabilityMap() {

  // Check that we have an elevation map for all the sensors
  for(int i = 0; i < n_sensors_; ++i) {
    if(!got_elevation_map_[i]) {
      ROS_WARN_STREAM("Layer " << i << " does not have an elevation map yet. "
                      "This may happen at initialization");
      return false;
    }
  }

  // Check that the maps have the same resolution. If not, we will not
  // compute the traversability because we need to perform a lot of
  // additional expensive checks on the matrices
  double resolution = elevation_grid_maps_[0].getResolution();
  for(int i = 1; i < n_sensors_; ++i) {
    if(resolution != elevation_grid_maps_[i].getResolution()) {
      ROS_ERROR("The elevation maps have different resolution. Traversability"
                " will not be computed");
      return false;
    }
  }

  // Extract the elevation data from the structure and fuse them. Here we
  // check that the maps have the same size. At the moment, maps with
  // different sizes are not supported
  std::vector<grid_map::Matrix> elevation_data(n_sensors_);
  int max_cols, max_rows;
  int bigger_map_index;

  for(int i = 0; i < n_sensors_; ++i) {
    elevation_data[i] = elevation_grid_maps_[i].get("elevation");
    if(i == 0 || max_cols < elevation_data[i].cols()) {
      max_cols = elevation_data[i].cols();
      bigger_map_index = i;
    }
    if(i == 0 || max_rows < elevation_data[i].rows()) {
      max_rows = elevation_data[i].rows();
      bigger_map_index = i;
    }
  }

  for(int i = 0; i < n_sensors_; ++i) {
    const int n_rows = elevation_data[i].rows();
    const int n_cols = elevation_data[i].cols();
    int delta_cols = max_cols - n_cols;
    int delta_rows = max_rows - n_rows;

    if(delta_cols > 0 || delta_rows > 0) {
      ROS_WARN_THROTTLE(5, "Current implementation cannot support maps with "
                        "different sizes in the proper way!");
      grid_map::Matrix padded_map(Eigen::MatrixXf::Zero(max_rows, max_cols));
      for(int r = 0; r < max_rows; ++r) {
        for (int c = 0; c < max_cols; ++c) {
          padded_map(r,c) = std::numeric_limits<float>::quiet_NaN();
        }
      }
      padded_map.block(delta_rows/2, delta_cols/2, n_rows, n_cols) =
              elevation_data[i];
      elevation_data[i] = padded_map;
    }
  }

  // Here we fuse the elevation maps from different sensor. We initialize the
  // fused map with the one from the first sensor (because for sure there is
  // at least one sensor). Then all the information will be overwritten by
  // the fusion. Just to be safe, we delete the elevation layer from the map.
  grid_map::GridMap fused_elevation_map;
  fused_elevation_map = elevation_grid_maps_[bigger_map_index];
  if(fused_elevation_map.exists("elevation")) {
    fused_elevation_map.clear("elevation");
  }

  // TODO(lucaBartolomei) At the moment need to iterate over values - better
  //  solution?
  grid_map::Matrix fused_elevation_matrix(Eigen::MatrixXf::Zero(
          max_rows, max_cols));
  for(int r = 0; r < max_rows; ++r) {
    for(int c = 0; c < max_cols; ++c) {
      double elevation_value = weights_[0] * elevation_data[0](r,c);
      for(int s = 1; s < n_sensors_; ++s) {
        if(std::isnan(elevation_value) &&
             !std::isnan(elevation_data[s](r,c))) {
          elevation_value = weights_[s] * elevation_data[s](r,c);
        } else if(!std::isnan(elevation_value) &&
             !std::isnan(elevation_data[s](r,c))) {
          elevation_value += weights_[s] * elevation_data[s](r,c);
        }
      }
      fused_elevation_matrix(r,c) = elevation_value / weights_sum_;
    }
  }

  // Add the elevation layer and whatever is missing
  fused_elevation_map.add("elevation", fused_elevation_matrix);
  for (const auto &layer : elevation_map_layers_) {
    if (!fused_elevation_map.exists(layer)) {
      fused_elevation_map.add(layer, 0.0);
    }
  }

  // Compute traversability from here
  grid_map_msgs::GridMap fused_elevation_maps_msg;
  grid_map::GridMapRosConverter::toMessage(fused_elevation_map,
          fused_elevation_maps_msg);
  traversability_map_->setElevationMap(fused_elevation_maps_msg);

  try {
    if (!traversability_map_->computeTraversability()) {
      ROS_WARN("Traversability Estimation: cannot compute traversability!");
      return false;
    }
    return true;
  } catch(std::exception &exception) {
    ROS_ERROR("Fusion of elevation maps exception: %s", exception.what());
    return false;
  }
}

} // end namespace smb_planner
