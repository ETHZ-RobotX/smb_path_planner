/**
 * @author Luca Bartolomei, V4RL
 * @brief  Main executable that runs the global planning algorithm.
 * @date   13.06.2019
 */

#include "smb_global_planner/planner/smb_global_planner.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "smb_global_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  smb_planner::PlannerParameters parameters;
  if (!smb_planner::readPlannerParameters(nh_private, parameters)) {
    ROS_ERROR("[Smb Global Planner] Could not read the parameters from "
              "server. Aborting...");
    return -1;
  }

  smb_global_planner::SmbGlobalPlanner node(nh, nh_private, parameters);
  ROS_INFO("[Smb Global Planner] Initialized SMB Global Planner with Voxblox.");

  ros::spin();
  return 0;
}
