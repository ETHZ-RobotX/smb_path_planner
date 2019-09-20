/**
 * @author Luca Bartolomei, V4RL
 * @date 13.06.2019
 */

#include "smb_planner_common/utility_visualization.h"

namespace smb_planner {
namespace utility_visualization {

visualization_msgs::Marker createMarkerForPathWithTraversability(
    const std::vector<Eigen::Vector3d> &path, const std_msgs::ColorRGBA &color,
    const std::string &name, const double scale, const std::string &frame_id,
    const double planning_height, const grid_map::GridMap &traversability_map,
    const double traversability_threshold,
    const double maximum_difference_elevation) {

  visualization_msgs::Marker path_marker;
  const int kPublishEveryNSamples = 1;
  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.color = color;
  path_marker.ns = name;
  path_marker.scale.x = scale;

  path_marker.points.reserve(path.size() / kPublishEveryNSamples);
  int i = 0;
  for (const Eigen::Vector3d &point : path) {
    i++;
    if (i % kPublishEveryNSamples != 0) {
      continue;
    }

    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(point, point_msg);
    point_msg.z = planning_height;

    // Modify if we have traversability information if available
    Eigen::Vector3d point_transf;
    if (utility_mapping::checkTraversabilityPosition(
            traversability_map, point.head<2>(), planning_height,
            traversability_threshold, maximum_difference_elevation,
            point_transf)) {
      tf::pointEigenToMsg(point_transf, point_msg);
    }
    path_marker.points.push_back(point_msg);
  }

  return path_marker;
}

visualization_msgs::Marker
createMarkerForPath(const std::vector<Eigen::Vector3d> &path,
                    const std_msgs::ColorRGBA &color, const std::string &name,
                    const double scale, const std::string &frame_id,
                    const double planning_height) {

  visualization_msgs::Marker path_marker;
  const int kPublishEveryNSamples = 1;
  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.color = color;
  path_marker.ns = name;
  path_marker.scale.x = scale;

  path_marker.points.reserve(path.size() / kPublishEveryNSamples);
  int i = 0;
  for (const Eigen::Vector3d &point : path) {
    i++;
    if (i % kPublishEveryNSamples != 0) {
      continue;
    }

    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(point, point_msg);
    point_msg.z = planning_height;
    path_marker.points.push_back(point_msg);
  }
  return path_marker;
}

visualization_msgs::Marker createMarkerForWaypointsWithTraversability(
    const std::vector<Eigen::Vector3d> &path, const std_msgs::ColorRGBA &color,
    const std::string &name, const double scale, const std::string &frame_id,
    const double planning_height, const grid_map::GridMap &traversability_map,
    const double traversability_threshold,
    const double maximum_difference_elevation) {
  visualization_msgs::Marker path_marker;

  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  path_marker.color = color;
  path_marker.color.a = 0.75;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = scale;
  path_marker.scale.z = scale;

  path_marker.points.reserve(path.size());
  for (const Eigen::Vector3d &point : path) {
    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(point, point_msg);
    point_msg.z = planning_height;

    // Modify if we have traversability information if available
    Eigen::Vector3d point_transf;
    if (utility_mapping::checkTraversabilityPosition(
            traversability_map, point.head<2>(), planning_height,
            traversability_threshold, maximum_difference_elevation,
            point_transf)) {
      tf::pointEigenToMsg(point_transf, point_msg);
    }
    path_marker.points.push_back(point_msg);
  }
  return path_marker;
}

visualization_msgs::Marker createMarkerForWaypoints(
    const std::vector<Eigen::Vector3d> &path, const std_msgs::ColorRGBA &color,
    const std::string &name, const double scale, const std::string &frame_id,
    const double planning_height) {
  visualization_msgs::Marker path_marker;

  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  path_marker.color = color;
  path_marker.color.a = 0.75;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = scale;
  path_marker.scale.z = scale;

  path_marker.points.reserve(path.size());
  for (const Eigen::Vector3d &point : path) {
    geometry_msgs::Point point_msg;
    tf::pointEigenToMsg(point, point_msg);
    point_msg.z = planning_height;
    path_marker.points.push_back(point_msg);
  }
  return path_marker;
}

visualization_msgs::Marker createMarkerForStateWithTraversability(
    const Eigen::Vector3d &state, const std_msgs::ColorRGBA &color,
    const std::string &name, const double scale, const std::string &frame_id,
    const double planning_height, const grid_map::GridMap &traversability_map,
    const double traversability_threshold,
    const double maximum_difference_elevation) {

  visualization_msgs::Marker state_marker;
  state_marker.header.frame_id = frame_id;

  state_marker.header.stamp = ros::Time::now();
  state_marker.type = visualization_msgs::Marker::SPHERE;
  state_marker.color = color;
  state_marker.color.a = 0.75;
  state_marker.ns = name;
  state_marker.scale.x = scale;
  state_marker.scale.y = scale;
  state_marker.scale.z = scale;

  // Modify if we have traversability information if available
  Eigen::Vector3d point_transf;
  if (utility_mapping::checkTraversabilityPosition(
          traversability_map, state.head<2>(), planning_height,
          traversability_threshold, maximum_difference_elevation,
          point_transf)) {
  }

  tf::pointEigenToMsg(point_transf, state_marker.pose.position);
  state_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  return state_marker;
}

visualization_msgs::Marker
createMarkerForState(const Eigen::Vector3d &state,
                     const std_msgs::ColorRGBA &color, const std::string &name,
                     const double scale, const std::string &frame_id) {

  visualization_msgs::Marker state_marker;
  state_marker.header.frame_id = frame_id;

  state_marker.header.stamp = ros::Time::now();
  state_marker.type = visualization_msgs::Marker::SPHERE;
  state_marker.color = color;
  state_marker.color.a = 0.75;
  state_marker.ns = name;
  state_marker.scale.x = scale;
  state_marker.scale.y = scale;
  state_marker.scale.z = scale;

  tf::pointEigenToMsg(state, state_marker.pose.position);
  state_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  return state_marker;
}

visualization_msgs::Marker createArrowMarkerForStateWithTraversability(
    const Eigen::Vector3d &state, const std_msgs::ColorRGBA &color,
    const std::string &name, const double scale, const std::string &frame_id,
    const double planning_height, const grid_map::GridMap &traversability_map,
    const double traversability_threshold,
    const double maximum_difference_elevation) {
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::ARROW;
  path_marker.color = color;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = 0.5 * scale;
  path_marker.scale.z = 0.5 * scale;

  path_marker.pose.orientation = tf::createQuaternionMsgFromYaw(state(2));
  path_marker.pose.position.x = state(0);
  path_marker.pose.position.y = state(1);
  path_marker.pose.position.z = planning_height;

  Eigen::Vector3d point_transf;
  if (utility_mapping::checkTraversabilityPosition(
          traversability_map, state.head<2>(), planning_height,
          traversability_threshold, maximum_difference_elevation,
          point_transf)) {
    path_marker.pose.position.x = point_transf(0);
    path_marker.pose.position.y = point_transf(1);
    path_marker.pose.position.z = point_transf(2);
  }

  return path_marker;
}

visualization_msgs::Marker createArrowMarkerForState(
    const Eigen::Vector3d &state, const std_msgs::ColorRGBA &color,
    const std::string &name, const double scale, const std::string &frame_id,
    const double planning_height) {
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = frame_id;

  path_marker.header.stamp = ros::Time::now();
  path_marker.type = visualization_msgs::Marker::ARROW;
  path_marker.color = color;
  path_marker.ns = name;
  path_marker.scale.x = scale;
  path_marker.scale.y = 0.5 * scale;
  path_marker.scale.z = 0.5 * scale;

  path_marker.pose.orientation = tf::createQuaternionMsgFromYaw(state(2));
  path_marker.pose.position.x = state(0);
  path_marker.pose.position.y = state(1);
  path_marker.pose.position.z = planning_height;

  return path_marker;
}

visualization_msgs::Marker createSphereCollisionForStateWithTraversability(
    const Eigen::Vector3d &state, const std::string &name,
    const std::string &frame_id, const double robot_radius,
    const double planning_height, const grid_map::GridMap &traversability_map,
    const double traversability_threshold,
    const double maximum_difference_elevation) {

  visualization_msgs::Marker state_marker;
  state_marker.header.frame_id = frame_id;

  state_marker.header.stamp = ros::Time::now();
  state_marker.type = visualization_msgs::Marker::SPHERE;
  state_marker.color.a = 0.4;
  state_marker.color.r = 204.0 / 255.0;
  state_marker.color.g = 1.0;
  state_marker.color.b = 1.0;
  state_marker.ns = name;
  state_marker.scale.x = 2.0 * robot_radius;
  state_marker.scale.y = 2.0 * robot_radius;
  state_marker.scale.z = 2.0 * robot_radius;

  Eigen::Vector3d point_transf;
  utility_mapping::checkTraversabilityPosition(
      traversability_map, state.head<2>(), planning_height,
      traversability_threshold, maximum_difference_elevation, point_transf);

  tf::pointEigenToMsg(point_transf, state_marker.pose.position);
  state_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  return state_marker;
}

visualization_msgs::Marker createSphereCollisionForState(
    const Eigen::Vector3d &state, const std::string &name,
    const std::string &frame_id, const double robot_radius) {

  visualization_msgs::Marker state_marker;
  state_marker.header.frame_id = frame_id;

  state_marker.header.stamp = ros::Time::now();
  state_marker.type = visualization_msgs::Marker::SPHERE;
  state_marker.color.a = 0.4;
  state_marker.color.r = 204.0 / 255.0;
  state_marker.color.g = 1.0;
  state_marker.color.b = 1.0;
  state_marker.ns = name;
  state_marker.scale.x = 2.0 * robot_radius;
  state_marker.scale.y = 2.0 * robot_radius;
  state_marker.scale.z = 2.0 * robot_radius;

  tf::pointEigenToMsg(state, state_marker.pose.position);
  state_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  return state_marker;
}

visualization_msgs::Marker createBoundaries(
    const Eigen::Vector3d &lower, const Eigen::Vector3d &upper,
    const std::string &name, const std::string &frame_id) {

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = frame_id;
  
  line_marker.header.stamp = ros::Time::now();
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.color.a = 0.4;
  line_marker.color.r = 1.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.0;
  line_marker.ns = name;
  line_marker.scale.x = 0.2;
  line_marker.scale.y = 0.0;
  line_marker.scale.z = 0.0;
 
  geometry_msgs::Point point_00;
  point_00.x = lower(0);
  point_00.y = lower(1);
  point_00.z = 0.0;
  
  geometry_msgs::Point point_10;
  point_10.x = upper(0);
  point_10.y = lower(1);
  point_10.z = 0.0;
  
  geometry_msgs::Point point_11;
  point_11.x = upper(0);
  point_11.y = upper(1);
  point_11.z = 0.0;
  
  geometry_msgs::Point point_01;
  point_01.x = lower(0);
  point_01.y = upper(1);
  point_01.z = 0.0;
  
  line_marker.points.push_back(point_00);
  line_marker.points.push_back(point_10);
  line_marker.points.push_back(point_11);
  line_marker.points.push_back(point_01);
  line_marker.points.push_back(point_00);

  return line_marker;
}

} // end namespace utility_visualization
} // end namespace smb_planner
