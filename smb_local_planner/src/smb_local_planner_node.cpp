/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that runs the local planning algorithm.
 * @date   14.06.2019
 */

#include "smb_local_planner/smb_local_planner.h"

int main(int argc, char **argv) {

  // Set up ROS
  ros::init(argc, argv, "smb_local_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Read parameters
  smb_planner::PlannerParameters params;
  if (!smb_planner::readPlannerParameters(nh_private, params)) {
    ROS_ERROR("[Smb Local Planner] Could not read the parameters from "
              "server. Aborting...");
    return -1;
  }

  // Set up node
  smb_local_planner::SmbLocalPlanner local_planner(nh, nh_private, params);
  ROS_INFO("[Smb Local Planner] Initialized SMB local planner");

  ros::spin();
  return 0;
}
