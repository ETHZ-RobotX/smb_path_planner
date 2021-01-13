cmake_minimum_required(VERSION 2.8.3)
project(smb_navigation_scripts)

# Find packages
find_package(catkin REQUIRED
  COMPONENTS
      rospy
      geometry_msgs
      nav_msgs
      sensor_msgs
)

catkin_package()

# Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)
