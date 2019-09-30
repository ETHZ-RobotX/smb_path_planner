/**
  * @author Luca Bartolomei, V4RL
  * @date   13.06.2019
  */

#include "smb_planner_common/utility_mapping.h"

namespace smb_planner {
namespace utility_mapping {

bool getNormalVectorFromGridMap(const grid_map_msgs::GridMap &map,
                                const int normal_index_x,
                                const int normal_index_y,
                                const int normal_index_z, Eigen::Vector3d &n) {

  double n_x = 0.0;
  double n_y = 0.0;
  double n_z = 0.0;
  int valid_positions = 0;

  for (int i = 0; i < map.data[0].data.size(); ++i) {
    // If it is not a nan, we can calculate the traversability there
    if (!std::isnan(map.data[normal_index_x].data[i])) {
      n_x += std::fabs(map.data[normal_index_x].data[i]);
      n_y += std::fabs(map.data[normal_index_y].data[i]);
      n_z += std::fabs(map.data[normal_index_z].data[i]);
      valid_positions++;
    }
  }

  if (valid_positions == 0) {
    return false; // ie the position may not be observed yet
  }

  // Get the normal to the surface
  n_x /= valid_positions;
  n_y /= valid_positions;
  n_z /= valid_positions;

  // Output
  n = Eigen::Vector3d(n_x, n_y, n_z);
  return true;
}

bool getTraversabilityValueFromGridMap(const grid_map_msgs::GridMap &map,
                                       const int traversability_index,
                                       double *traversability_val) {

  int valid_positions = 0;

  // Iterate over the cells in the traversability layer
  for (int i = 0; i < map.data[traversability_index].data.size(); ++i) {
    // If it is not a nan, we can calculate the traversability there
    if (!std::isnan(map.data[traversability_index].data[i])) {
      *traversability_val += std::fabs(map.data[traversability_index].data[i]);
      valid_positions++;
    }
  }

  if (valid_positions == 0) {
    return false;
  } else {
    *traversability_val /= double(valid_positions);
    return true;
  }
}

bool getElevationValueFromGridMap(const grid_map_msgs::GridMap &map,
                                  const int elevation_index,
                                  double *elevation) {

  int valid_positions = 0;

  // Iterate over the cells in the elevation layer
  for (int i = 0; i < map.data[elevation_index].data.size(); ++i) {
    // If it is not a nan, we can calculate the elevation there
    if (!std::isnan(map.data[elevation_index].data[i])) {
      *elevation += std::fabs(map.data[elevation_index].data[i]);
      valid_positions++;
    }
  }

  if (valid_positions == 0) {
    return false;
  } else {
    *elevation /= double(valid_positions);
    return true;
  }
}

TraversabilityStatus getTrasversabilityInformation(
    const grid_map::GridMap &traversability_map,
    const Eigen::Vector2d &position, const double projection_distance,
    const double traversability_threshold,
    const double maximum_difference_elevation, Eigen::Vector3d &projection) {

  // Initialize output
  projection = Eigen::Vector3d(position(0), position(1), projection_distance);

  // Request information - select dimension of 1.0 m around position
  grid_map::Position requestedSubmapPosition(position(0), position(1));
  grid_map::Length requestedSubmapLength(1.0, 1.0);

  bool isSuccess;
  grid_map_msgs::GridMap result_map;
  grid_map::GridMap subMap = traversability_map.getSubmap(
      requestedSubmapPosition, requestedSubmapLength, isSuccess);
  grid_map::GridMapRosConverter::toMessage(subMap, result_map);

  // First case: the sample is outside the local map.
  if (!isSuccess) {
    return TraversabilityStatus::UNKNOWN;
  }

  // Second case: we are in the local map
  int traversability_index = 0;
  int elevation_index = 0;
  int normal_index_x = 0, normal_index_y = 0, normal_index_z = 0;

  // Get the valid indexes for every info we need
  for (int i = 0; i < result_map.layers.size(); ++i) {
    if (result_map.layers[i].compare("traversability") == 0) {
      traversability_index = i;
    }
    if (result_map.layers[i].compare("elevation") == 0) {
      elevation_index = i;
    }
    if (result_map.layers[i].compare("surface_normal_x") == 0) {
      normal_index_x = i;
    }
    if (result_map.layers[i].compare("surface_normal_y") == 0) {
      normal_index_y = i;
    }
    if (result_map.layers[i].compare("surface_normal_z") == 0) {
      normal_index_z = i;
    }
  }

  // Get values
  double traversability_val = 0.0;
  double elevation = 0.0;
  double n_x = 0.0, n_y = 0.0, n_z = 0.0;

  int valid_positions = 0;

  // Iterate over the data in the grid map
  for (int i = 0; i < result_map.data[elevation_index].data.size(); ++i) {
    // If it is not a nan, we can calculate the elevation there
    if (!std::isnan(result_map.data[traversability_index].data[i]) &&
        !std::isnan(result_map.data[elevation_index].data[i]) &&
        !std::isnan(result_map.data[normal_index_x].data[i]) &&
        !std::isnan(result_map.data[normal_index_y].data[i]) &&
        !std::isnan(result_map.data[normal_index_z].data[i])) {

      traversability_val +=
          std::fabs(result_map.data[traversability_index].data[i]);
      elevation += std::fabs(result_map.data[elevation_index].data[i]);
      n_x += std::fabs(result_map.data[normal_index_x].data[i]);
      n_y += std::fabs(result_map.data[normal_index_y].data[i]);
      n_z += std::fabs(result_map.data[normal_index_z].data[i]);
      valid_positions++;
    }
  }

  // We don't have valid data - we consider the position unknown
  if (valid_positions == 0) {
    return TraversabilityStatus::UNKNOWN;
  }

  // Check if we are nearby a cliff (ie if the distance between the
  // minimum and maximum elevation is higher than a threshold)
  std::vector<float>::iterator min_elevation, max_elevation;
  min_elevation =
      std::min_element(result_map.data[elevation_index].data.begin(),
                       result_map.data[elevation_index].data.end());
  max_elevation =
      std::max_element(result_map.data[elevation_index].data.begin(),
                       result_map.data[elevation_index].data.end());

  if (!std::isnan(*max_elevation) && !std::isnan(*min_elevation) &&
      (*max_elevation - *min_elevation) >
          1.2 * static_cast<float>(maximum_difference_elevation)) {
    return TraversabilityStatus::UNTRAVERSABLE;
  }

  // Compute the real values
  traversability_val /= double(valid_positions);
  elevation /= double(valid_positions);
  n_x /= double(valid_positions);
  n_y /= double(valid_positions);
  n_z /= double(valid_positions);

  // We have valid data: we can compute if the position is traversable
  if (traversability_val >= traversability_threshold) {
    projection = Eigen::Vector3d(position(0), position(1), elevation) +
                 projection_distance * Eigen::Vector3d(n_x, n_y, n_z);
    return TraversabilityStatus::TRAVERSABLE;
  }
  // Else: it is pointless to project the position in this case
  return TraversabilityStatus::UNTRAVERSABLE;
}

bool checkTraversabilityMultiplePositions(
    const grid_map::GridMap &traversability_map,
    const std::vector<Eigen::Vector2d> &positions,
    const double projection_distance, const double traversability_threshold,
    const double maximum_difference_elevation,
    std::vector<Eigen::Vector3d> &projections,
    std::vector<TraversabilityStatus> &status) {

  // Clear the containers
  projections.clear();
  projections.resize(positions.size());

  status.clear();
  status.resize(positions.size());

  // Get the information for every position
  for (int i = 0; i < positions.size(); ++i) {
    status[i] = getTrasversabilityInformation(
        traversability_map, positions[i], projection_distance,
        traversability_threshold, maximum_difference_elevation, projections[i]);

    // If it is untraversable, then we can directly return
    if (status[i] == TraversabilityStatus::UNTRAVERSABLE) {
      return false;
    }
  }

  return true;
}

bool checkTraversabilityPosition(const grid_map::GridMap &traversability_map,
                                 const Eigen::Vector2d &position,
                                 const double projection_distance,
                                 const double traversability_threshold,
                                 const double maximum_difference_elevation,
                                 Eigen::Vector3d &projection) {

  TraversabilityStatus traversability_status = getTrasversabilityInformation(
      traversability_map, position, projection_distance,
      traversability_threshold, maximum_difference_elevation, projection);

  if (traversability_status == TraversabilityStatus::TRAVERSABLE) {
    return true;
  } else {
    return false;
  }
}

void getSphereAroundPoint(const voxblox::Layer<voxblox::TsdfVoxel> &layer,
                          const voxblox::Point &center,
                          voxblox::FloatingPoint radius,
                          voxblox::HierarchicalIndexMap *block_voxel_list) {

  CHECK_NOTNULL(block_voxel_list);
  float voxel_size = layer.voxel_size();
  float voxel_size_inv = 1.0 / layer.voxel_size();
  int voxels_per_side = layer.voxels_per_side();

  const voxblox::GlobalIndex center_index =
      voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(center,
                                                           voxel_size_inv);
  const voxblox::FloatingPoint radius_in_voxels = radius / voxel_size;

  for (voxblox::FloatingPoint x = -radius_in_voxels; x <= radius_in_voxels;
       x++) {
    for (voxblox::FloatingPoint y = -radius_in_voxels; y <= radius_in_voxels;
         y++) {

      voxblox::Point point_voxel_space(x, y, center.z() / voxel_size);

      // check if point is inside the spheres radius
      // TODO (lucaBartolomei) note that this check on Voxblox and the robot
      // radius has been removed because it did not allow to check for collision
      // some parts of the global path. Notice that it is there in the original
      // Voxblox implementation!!
      //if (point_voxel_space.head(2).norm() <= radius_in_voxels) {
        voxblox::GlobalIndex voxel_offset_index(
            std::floor(point_voxel_space.x()),
            std::floor(point_voxel_space.y()),
            std::floor(point_voxel_space.z()));
        // Get the block and voxel indices from this.
        voxblox::BlockIndex block_index;
        voxblox::VoxelIndex voxel_index;

        voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
            voxel_offset_index + center_index, voxels_per_side, &block_index,
            &voxel_index);
        (*block_voxel_list)[block_index].push_back(voxel_index);
      //}
    }
  }
}

bool rampInterpolatorWaypoints(
    const std::vector<Eigen::Vector3d> &waypoints,
    std::vector<Eigen::VectorXd> &interpolated_waypoints, const double v_max,
    const double a_max, const double sampling_dt) {

  // The waypoint list is empty --> unsuccessful interpolation
  if (waypoints.empty()) {
    return false;
  }

  // Initialize variables
  double time = 0.0;
  double velocity = 0.0;

  // Iterate over pair of waypoints
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    // Extract start and end point for the current segment
    Eigen::Vector2d start = waypoints[i].head(2);
    Eigen::Vector2d end = waypoints[i + 1].head(2);

    // Extract the current yaw
    Eigen::Vector2d direction = (end - start).normalized();
    double yaw = std::atan2(direction(1), direction(0));

    // Figure out what the total segment time will be.
    double total_segment_distance = (end - start).norm();
    // Total time needed to get to max speed (or go from max speed to 0)
    double min_acceleration_time = v_max / a_max;
    // The amount of distance covered during the acceleration
    // (or decceleration process).
    double min_acceleration_distance =
        v_max * min_acceleration_time -
        0.5 * a_max * min_acceleration_time * min_acceleration_time;

    double total_segment_time = 0.0;
    // Case 1: time is shorter than the acceleration and
    // decceleration time.
    if (total_segment_distance < 2 * min_acceleration_distance) {
      total_segment_time = 2 * std::sqrt(total_segment_distance / a_max);
    } else {
      // Case 2: time is longer than accel + deccel time.
      total_segment_time =
          2 * min_acceleration_time +
          (total_segment_distance - 2 * min_acceleration_distance) / v_max;
    }
    size_t num_elements = total_segment_time / sampling_dt;
    Eigen::Vector2d path_direction = (end - start).normalized();

    // Treat this as a 1D problem since it is. ;)
    double position = 0.0;

    // Separate the time between total time and local segment time
    double current_time_trajectory = time;
    int64_t current_time_segment = 0.0;

    for (size_t j = 0; j < num_elements; ++j) {
      // Integrate velocity to get position.
      position += velocity * sampling_dt;

      // Figure out if we're accelerating, deccelerating, or neither.
      // Handle Case 1 first:
      if (total_segment_time < min_acceleration_time * 2) {
        if (current_time_segment < total_segment_time / 2.0) {
          velocity += a_max * sampling_dt;
        } else {
          velocity -= a_max * sampling_dt;
        }
      } else {
        // Case 2
        if (position <= min_acceleration_distance) {
          velocity += a_max * sampling_dt;
        } else if ((total_segment_distance - position) <=
                   min_acceleration_distance) {
          velocity -= a_max * sampling_dt;
        }
      }

      // Make sure to meet constraints (could be passed/missed due to
      // discretization error).
      if (position > total_segment_distance) {
        position = total_segment_distance;
      }
      if (velocity > v_max) {
        velocity = v_max;
      }
      if (velocity < 0) {
        velocity = 0;
      }

      // Save the interpolated path
      Eigen::Vector2d posit(start + path_direction * position);
      double z_interp =
          waypoints[i](2) +
          (waypoints[i].head(2) - posit).norm() /
              (waypoints[i].head(2) - waypoints[i + 1].head(2)).norm() *
              (waypoints[i + 1](2) - waypoints[i](2));

      Eigen::VectorXd point(5);
      point << posit(0), posit(1), z_interp, yaw, current_time_trajectory;
      interpolated_waypoints.push_back(point);

      // Update times
      current_time_trajectory += sampling_dt;
      current_time_segment += sampling_dt;
    }

    time = current_time_trajectory + sampling_dt;
  }

  // Add end point
  Eigen::Vector2d position_end = waypoints.back().head(2);
  Eigen::Vector2d direction_end =
      (position_end - waypoints[waypoints.size() - 2].head(2)).normalized();
  double yaw_end = std::atan2(direction_end(1), direction_end(0));
  double z_end =
      waypoints[waypoints.size() - 2](2) +
      (waypoints[waypoints.size() - 2].head(2) - position_end).norm() /
          (waypoints[waypoints.size() - 2].head(2) - waypoints.back().head(2))
              .norm() *
          (waypoints.back()(2) - waypoints[waypoints.size() - 2](2));

  Eigen::VectorXd point_end(5);
  point_end << position_end(0), position_end(1), z_end, yaw_end, time;
  interpolated_waypoints.push_back(point_end);
  return true;
}

bool interpolateInitialRotation(
    std::vector<Eigen::VectorXd> &interpolated_waypoints, 
    const Eigen::Vector3d &current_state, const double target_yaw,
    const double v_max, const double sampling_dt, 
    const double max_initial_rotation, const double nominal_height) {

  // Check if we need to add rotation; if not, just return
  if(std::fabs(current_state(2) - target_yaw) >= max_initial_rotation) {

    double yaw = current_state(2);
    double time = 0.0;
  
    // Create vector of initial rotation commands
    double delta = target_yaw - current_state(2);
    if(delta > M_PI)
        delta -= 2.0 * M_PI;
    if(delta < -M_PI)
        delta += 2.0 * M_PI;
    size_t num_elements = std::fabs(delta) / (v_max * sampling_dt);
    std::vector<Eigen::VectorXd> rotation_vector(num_elements);

    for(size_t i=0; i<num_elements; ++i) {
        yaw += delta / num_elements;
        if(yaw > 2.0*M_PI)
          yaw -= 2.0*M_PI;
        if(yaw < -2.0*M_PI)
          yaw += 2.0*M_PI;
             
        Eigen::VectorXd waypoint(5); 
        waypoint << current_state(0), current_state(1), 
                    nominal_height, yaw, time;
        rotation_vector[i] = waypoint;
        time += sampling_dt;
    }
    
    if(!rotation_vector.empty()) { // ie num_elements != 0
      // Insert the rotation command vector to the beginning of the interpolated
      // waypoints vector
      interpolated_waypoints.insert(interpolated_waypoints.begin(),
                                  rotation_vector.begin(), 
                                  rotation_vector.end()); 
      // Re-timing of the interpolated waypoints to account for new commands of 
      // initial rotation (i.e. shift all the timings by the initial offset)
      for(size_t i = num_elements; i < interpolated_waypoints.size(); ++i) {
        interpolated_waypoints[i](4) += rotation_vector.back()(4) + sampling_dt;
      }
    }
    return true;
  } else {
    return false;
  }    
}

void createYawsFromStates(const std::vector<Eigen::Vector2d> &states,
                          std::vector<double> &yaws) {

  // Assure that the output vector is empty
  yaws.clear();
  yaws.resize(states.size(), 0.0);

  // Start filling the vector
  for (size_t i = 0; i < states.size() - 1; ++i) {
    Eigen::Vector2d dir =
        (states[i + 1].head<2>() - states[i].head<2>()).normalized();
    yaws[i] = std::atan2(dir(1), dir(0));
  }
  yaws.back() = yaws[yaws.size() - 2];
}

double pi2pi(const double angle) {

  double ang;
  /* Process angle*/
  if ((angle < -2 * M_PI) || (angle > 2 * M_PI)) {
    ang = fmod(angle, 2 * M_PI);
  } else {
    ang = angle;
  }

  if (ang > M_PI)
    ang = ang - 2 * M_PI;
  if (ang < -M_PI)
    ang = ang + 2 * M_PI;
  return ang;
}

} // end namespace utility_mapping
} // end namespace smb_planner
