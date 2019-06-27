/**
  * @author Luca Bartolomei, V4RL
  * @brief  Main class for global planning using OMPL, voxblox and
  *         traversability information
  * @date   13.06.2019
  */

#pragma once

#include <memory>
#include <string>

#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <smb_planner_common/planner_parameters.h>
#include <smb_planner_common/traversability_estimator.h>
#include <std_srvs/Empty.h>
#include <voxblox_ros/esdf_server.h>
#include <smb_msgs/SmbState.h>
#include <smb_planner_msgs/PlannerService.h>

#include "smb_global_planner/planner/voxblox_ompl_rrt.h"

using namespace smb_planner;

namespace smb_global_planner {

class SmbGlobalPlanner {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
  * @brief Class constructor
  * @param[in] nh : ROS node handle
  * @param[in] nh_private : Private ROS node handle
  * @param[in] params : Parameters useful for planning
  */
  SmbGlobalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                   const PlannerParameters &params);

  /**
  * @brief Destructor
  */
  ~SmbGlobalPlanner();

private:
  /**
   * @brief Method that initializes ROS-related variables and communications
   */
  void initROS();

  /**
   * @brief Set of callbacks for subscribers and services. Each callback
   *        receives as an input a specific type of message
   */
  void stateEstimationCallback(const geometry_msgs::PoseStamped &pose_msg);

  void smbStateCallback(const smb_msgs::SmbState &smb_state_msg);

  bool plannerServiceCallback(smb_planner_msgs::PlannerService::Request &req,
                              smb_planner_msgs::PlannerService::Response &res);

  /**
   * @brief Main function for the planner: this method is repeated by a ROS
   *        timer at constant rate
   * @param[in] event : Event triggering the callback
   */
  void plannerTimerCallback(const ros::TimerEvent &event);

  /**
   * @brief Method that computes the map bounds from the voxblox maps
   * @param[out] lower_bound : lower bounds of the map
   * @param[out] upper_bound : upper bounds of the map
   */
  void computeMapBounds(Eigen::Vector3d *lower_bound,
                        Eigen::Vector3d *upper_bound) const;

  /**
   * @brief Method that returns the signed distance of the query position to
   *        the closest obstacle
   * @param[in] position : query position to check
   * @return Signed distance value to the closest obstacle
   */
  double getMapDistance(const Eigen::Vector3d &position) const;

  /**
   * @brief Method that checks if a query point is in free space or not. This
   *        function uses the TSDF information.
   * @param[in] robot_position : query position
   * @return True if the position is not in collision, False otherwise
   */
  bool checkOptimisticMapCollision(const Eigen::Vector3d &robot_position);

  /**
   * @brief Method that computes the length of the input path
   * @param[in] waypoints : input path
   * @return Length of the path
   */
  double computePathLength(const std::vector<Eigen::Vector3d> &waypoints) const;

  /**
   * @brief Method that publishes the current global path to the local planner
   */
  void publishTrajectory() const;

  /**
   * @brief Method that sends the stop commands directly to the MPC
   */
  void sendStopCommand() const;

  /**
   * @brief Utility function that checks if the current global path will not
   * bring
   *        to collisions.
   * @return True if the path does not bring to collision, False otherwise
   */
  bool isPathCollisionFree();

  /**
   * @brief Method that checks if the current global goal lies in free space
   * @return True if the goal is valid, False otherwise
   */
  bool isGoalValid();

  /**
   * @brief Method that checks if the robot has reached the global goal
   */
  void checkDistanceGoal();

  /**
   * @brief Method that create a line strip marker for a path. It considers
   *        the traversability information if available
   * @param[in] path : query path to show
   * @param[in] color : color of the marker
   * @param[in] name : namespace of the marker
   * @param[in] scale : dimension of the marker
   * @return The marker for line strip visualization
   */
  visualization_msgs::Marker
  createMarkerForPath(const std::vector<Eigen::Vector3d> &path,
                      const std_msgs::ColorRGBA &color, const std::string &name,
                      double scale) const;

  /**
   * @brief Method that create a sphere list marker for a path. It considers
   *        the traversability information if available
   * @param[in] path : query path to show
   * @param[in] color : color of the marker
   * @param[in] name : namespace of the marker
   * @param[in] scale : dimension of the marker
   * @return The marker for sphere list visualization
   */
  visualization_msgs::Marker
  createMarkerForWaypoints(const std::vector<Eigen::Vector3d> &path,
                           const std_msgs::ColorRGBA &color,
                           const std::string &name, double scale);

  /**
   * @brief Method that create a sphere marker for a state. It considers
   *        the traversability information if available
   * @param[in] state : query state to show
   * @param[in] color : color of the marker
   * @param[in] name : namespace of the marker
   * @param[in] scale : dimension of the marker
   * @return The marker for sphere visualization
   */
  visualization_msgs::Marker
  createMarkerForState(const Eigen::Vector3d &state,
                       const std_msgs::ColorRGBA &color,
                       const std::string &name, double scale);

  /**
   * @brief Method that create an arrow marker for a state. It considers
   *        the traversability information if available
   * @param[in] state : query state to show
   * @param[in] color : color of the marker
   * @param[in] name : namespace of the marker
   * @param[in] scale : dimension of the marker
   * @return The marker for arrow visualization
   */
  visualization_msgs::Marker
  createArrowMarkerForState(const Eigen::Vector3d &state,
                            const std_msgs::ColorRGBA &color,
                            const std::string &name, double scale) const;

  /**
   * @brief Method that create a sphere marker around a state to show the
   *        collision checking. It considers the traversability information
   *        if available
   * @param[in] state : query state where to plot the sphere
   * @param[in] name : namespace of the marker
   * @return The marker for sphere visualization
   */
  visualization_msgs::Marker
  createBoundingBoxForState(const Eigen::Vector3d &state,
                            const std::string &name) const;

  /**
   * @brief Method that delete all the old marker to make visualization less
   *        messy
   */
  void clearAllOldMarkers();

  /**
   * @brief Method to publish the visualization of all the markers for a
   *        series of waypoints
   * @param[in] waypoints : series of waypoints where to plot the markers
   */
  void publishVisualization(const std::vector<Eigen::Vector3d> &waypoints);

  /**
   * @brief Method to publish the visualization of all the markers for a
   *        series of waypoints
   * @param[in] waypoints : series of waypoints where to plot the markers
   */
  void publishVisualization(const std::vector<Eigen::VectorXd> &waypoints);

  /**
   * @brief Method to publish the visualization of all the markers
   */
  void publishVisualization();

protected:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber state_estimation_sub_;
  ros::Subscriber smb_state_sub_;

  ros::Publisher path_marker_pub_;
  ros::Publisher trajectory_pub_;

  ros::ServiceServer planner_srv_;

  // ROS Timers
  ros::Timer planning_timer_;
  ros::CallbackQueue planning_queue_;
  ros::AsyncSpinner planning_spinner_;

  // Containers
  Eigen::Vector3d current_state_;
  Eigen::Vector3d goal_;
  std::vector<Eigen::Vector3d> waypoints_;
  std::vector<Eigen::VectorXd> interpolated_waypoints_;

  // Check variables
  bool perform_planning_;
  bool has_state_info_;
  bool optimistic_;

  // Helper for updating the visualization of the path
  int n_iter_vis_;

  // Visualization container
  visualization_msgs::MarkerArray marker_array_;

  // Parameters
  PlannerParameters params_;

  // Map!
  voxblox::EsdfServer voxblox_server_;
  // Shortcuts to the maps:
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::TsdfMap::Ptr tsdf_map_;

  // Traversability information
  std::unique_ptr<TraversabilityEstimator> traversability_estimator_;

  // Planner
  VoxbloxOmplRrt rrt_;
};

} // namespace smb_global_planner
