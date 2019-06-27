/**
  * @author Luca Bartolomei, V4RL
  * @brief  Main class for local planning. This is a wrapper around the
  *         actual CHOMP solver. We can also use traversability information
  *         during planning.
  * @date   14.06.2019
  */

#pragma once

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <smb_planner_common/utility_visualization.h>
#include <voxblox_ros/esdf_server.h>
#include <smb_msgs/SmbState.h>

#include "smb_local_planner/chomp_solver.h"

namespace smb_local_planner {

class SmbLocalPlanner {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Class constructor
   * @param[in] nh : ROS node handle
   * @param[in] nh_private : Private ROS node handle
   * @param[in] params : Parameters useful for planning
   */
  SmbLocalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                  const PlannerParameters &params);

  /**
   * @brief Destructor
   */
  ~SmbLocalPlanner();

private:
  /**
   * @brief Method that initializes ROS-related variables and communications
   */
  void initROS();

  /**
   * @brief Method that checks the current position to see if the global goal
   *        has been reached
   * @param[in] verbose : if we should print the message if the goal is reached
   * @return True if goal is reached, False otherwise
   */
  bool globalGoalReached(const bool verbose = false) const;

  /**
   * @brief Method that extract the local goal from the global path. If
   *        either uses the current local goal or select a new one. It also
   *        checks if the new waypoint is farther away than a threshold
   * @return Local goal position to reach in Eigen format (x, y, z)
   */
  Eigen::Vector3d extractLocalGoal();

  /**
   * @brief Method to extract the pose where the planner should plan from.
   *        It returns either the current state or gets the pose on the
   *        current local path a given number of seconds in the future
   * @return Start state in Eigen format (x, y, yaw)
   */
  Eigen::Vector3d getStartState() const;

  /**
   * @brief Method to interpolate the trajectory output from CHOMP using a
   *        ramp interpolator and store the result in the local path
   */
  void interpolateLocalPath();

  /**
   * @brief Iterate over the states in the path and check if any of them will
   *        move the robot in untraversable space (if traversability check
   *        is enabled) or to collision against obstacles
   * @param[in] path : sequence of states (x,y,yaw,time) to be checked
   * @return True if no collision/untraversability is detected, False
   *        otherwise
   */
  bool isPathCollisionFree(const std::vector<Eigen::Vector4d> &path) const;

  /**
   * @brief Method that returns the signed distance of the query position to
   *        the closest obstacle
   * @param[in] position : query position to check
   * @return Signed distance value to the closest obstacle
   */
  double getMapDistance(const Eigen::Vector3d &position) const;

  /**
   * @brief Method that returns the signed distance of the query position to
   *        the closest obstacle and the corresponding gradient in the ESDF
   * @param[in] position : query position to check
   * @param[out] gradient : gradient of the ESDF signed distance field in
   *        the query position
   * @return Signed distance value to the closest obstacle
   */
  double getMapDistanceAndGradient(const Eigen::Vector3d &position,
                                   Eigen::Vector3d *gradient) const;

  /**
   * @brief Method to find an intermediate position goal between a start
   *        and a goal positions, by checking for a step size distance. It
   *        enforces the new goal to be in free space
   * @param[in] start_point : start position where to start the checks
   * @param[in] goal_point : candidate goal position to be checked
   * @param[in] step_size : step size to get the nearest free space point
   * @param[out] goal_out : the update local goal position
   * @return True if the search of a new local goal is successful, False oth.
   */
  bool findIntermediateGoal(const Eigen::Vector2d &start_point,
                            const Eigen::Vector2d &goal_point, double step_size,
                            Eigen::Vector2d *goal_out) const;

  /**
   * @brief Method to get the closest position to a query lying in free
   *        space by following the gradient information of the distance
   *        field using a fixed step size
   * @param[in] pos : query position to be checked
   * @param[in] step_size : distance to be used to follow the gradient
   * @param[out] new_pos : new updated position that lies in free space
   * @return True if search of new position in free space is successful,
   *        False otherwise
   */
  bool getNearestFreeSpaceToPoint(const Eigen::Vector2d &pos, double step_size,
                                  Eigen::Vector2d *new_pos) const;

  /**
   * @brief Method to send a stop command to the MPC. It resets the local
   *        planner as well
   */
  void sendStopCommand();

  /**
   * @brief Method to send rotation command to the MPC
   */
  void sendRotationCommand() const;

  /**
   * @brief Method that creates a command input to the controller to make
   *        the robot rotate on spot. It makes the robot rotate to face the
   *        first waypoint of the global path.
   * @return True if interpolation is successful, False otherwise
   */
  bool interpolateRotation();

  /**
   * @brief Method that checks if we commands in the buffer. If not, we can
   *        start planning from scratch. Otherwise, we can incrementally plan
   * @return True if we have commands in the buffer, False otherwise
   */
  bool hasValidCommands();

  /**
   * @brief Method that resets the local planner when a new global path is
   *        received
   */
  void resetLocalPlanner();

  /**
   * @brief Method to start sending the commands to the controller
   * @return True if the commands can be sent, False otherwise
   */
  bool startSendingCommands();

  /**
   * @brief Callbacks to trigger the planner
   * @return True if the planner can start, ie if we have state information
   *        and a global path
   */
  bool triggerServiceCallback(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res);

  /**
   * @brief Set of callbacks for subscribers. Each callback must receive
   *        as an input a specific type of message to get the state of the
   *        robot
   */
  void stateEstimationCallback(const geometry_msgs::PoseStamped &pose_msg);

  void smbStateCallback(const smb_msgs::SmbState &smb_state_msg);

  /**
   * @brief Callback to receive the global path from the global planner
   * @param[in] path_msg : message containing the global path
   */
  void globalPlannerCallback(const nav_msgs::PathConstPtr &path_msg);

  /**
   * @brief Main function that implements the planning strategy. This
   *        callback is called at a fixed frequency and computes the local
   *        path that will be sent to the controller. It extracts the
   *        starting and ending point for the local trajectory computation
   *        and plans using CHOMP only in free space
   * @param[in] event : ROS event trigger the timer
   */
  void plannerTimerCallback(const ros::TimerEvent &event);

  /**
   * @brief Main responsible for the communication with the controller.
   *        This callback is called at fixed time frequency and extracts
   *        the chunks of the local path to be tracked by the MPC
   * @param[in] event : ROS event trigger the timer
   */
  void commandPublishTimerCallback(const ros::TimerEvent &event);

  /**
   * @brief Method to publish the visualization of all the markers
   */
  void publishVisualization();

  /**
   * @brief Method to delete all old markers
   */
  void deleteAllMarkers();

  /**
   * @brief Method to create a sphere list for the CHOMP optimized trajectory
   * @return Marker for sphere list
   */
  visualization_msgs::Marker createWaypointsMarkers() const;

  /**
   * @brief Method to create a sphere list for the local path
   * @return Marker for sphere list
   */
  visualization_msgs::Marker createLocalPathLineStripMarkers() const;

  /**
   * @brief Method to create a line strip for the local path
   * @return Marker for line strip
   */
  visualization_msgs::Marker createPathMarkers() const;

protected:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber state_estimation_sub_;
  ros::Subscriber smb_state_sub_;
  ros::Subscriber global_planner_sub_;

  ros::Publisher path_marker_pub_;
  ros::Publisher trajectory_mpc_pub_;

  ros::ServiceServer trigger_local_srv_;

  // Timers
  ros::Timer planning_timer_;
  ros::CallbackQueue planning_queue_;
  ros::AsyncSpinner planning_spinner_;

  ros::Timer command_publishing_timer_;
  ros::CallbackQueue command_publishing_queue_;
  ros::AsyncSpinner command_publishing_spinner_;

  // Map
  voxblox::EsdfServer voxblox_server_;
  std::unique_ptr<TraversabilityEstimator> traversability_estimator_;

  // Shortcuts to the maps:
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::TsdfMap::Ptr tsdf_map_;

  // Auxiliaries
  PlannerParameters params_;
  Eigen::Vector3d current_state_;
  visualization_msgs::MarkerArray marker_array_;

  std::vector<Eigen::Vector3d> global_path_;
  std::vector<Eigen::Vector4d> local_path_;
  std::vector<Eigen::Vector4d> commands_;
  std::vector<Eigen::Vector4d> commands_rotation_;

  // Check variables
  bool has_state_info_;
  bool start_planning_;
  bool start_sending_commands_;
  bool rotate_;
  bool first_rotation_;

  // Indices
  int wp_num_;                  // Waypoint number to track global path
  size_t path_index_;           // Index of the buffer to publish command
  int iter_initialization_mpc_; // Check variable to start communication
                                // with MCP
  double initial_yaw_rot_;      // Initial yaw rotation to face in the
                                // direction of the global path

  // Mutex for path and command buffer access
  std::recursive_mutex commands_mutex_;

  // Planner
  std::unique_ptr<smb_local_planner::ChompSolver> chomp_solver_;
};

} // end namespace smb_local_planner
