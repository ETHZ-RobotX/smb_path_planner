#include <ros/ros.h>

#include "smb_navigation/OdometryTransformer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_transformer_node");
  ros::NodeHandle nh_private("~");

  smb_navigation::OdometryTransformer odometry_transformer(nh_private);

  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
