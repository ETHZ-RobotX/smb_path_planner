/*
 * @brief Interactive marker to select the global goal
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: 14.06.2019
 */

#pragma once

#include <functional>

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>

namespace smb_navigation_rviz
{

class PlanningInteractiveMarkers
{
public:
  typedef std::function<void(const geometry_msgs::Pose& pose)>
      PoseUpdatedFunctionType;

  PlanningInteractiveMarkers(const ros::NodeHandle& nh);
  ~PlanningInteractiveMarkers() {}

  void setFrameId(const std::string& frame_id);
  // Bind callback for whenever pose updates.

  void setPoseUpdatedCallback(const PoseUpdatedFunctionType& function)
  {
    pose_updated_function_ = function;
  }

  void initialize();

  // Functions to interface with the set_pose marker:
  void enableSetPoseMarker(const geometry_msgs::Pose& pose);
  void disableSetPoseMarker();
  void setPose(const geometry_msgs::Pose& pose);

  // Functions to interact with markers from the marker map (no controls):
  void enableMarker(const std::string& id, const geometry_msgs::Pose& pose);
  void updateMarkerPose(const std::string& id, const geometry_msgs::Pose& pose);
  void disableMarker(const std::string& id);

  void processSetPoseFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

private:
  // Creates markers without adding them to the marker server.
  void createMarkers();

  // ROS stuff.
  ros::NodeHandle nh_;
  interactive_markers::InteractiveMarkerServer marker_server_;

  // Settings.
  std::string frame_id_;

  // State.
  bool initialized_;
  visualization_msgs::InteractiveMarker set_pose_marker_;

  // This is map for waypoint visualization markers:
  std::map<std::string, visualization_msgs::InteractiveMarker> marker_map_;
  // This determines how the markers in the marker map will look:
  visualization_msgs::InteractiveMarker marker_prototype_;

  // State:
  PoseUpdatedFunctionType pose_updated_function_;
};

} // end namespace smb_navigation_rviz
