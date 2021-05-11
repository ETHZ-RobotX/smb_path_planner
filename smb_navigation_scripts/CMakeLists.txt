cmake_minimum_required(VERSION 2.8.3)
project(smb_navigation_scripts)

# Find packages
find_package(catkin REQUIRED
  COMPONENTS
      rospy
      geometry_msgs
      nav_msgs
      sensor_msgs
      pcl_ros
)

catkin_package()

# Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(filter_pcl_node
  src/filter_pcl_node.cpp
)

target_link_libraries(filter_pcl_node
   ${catkin_LIBRARIES}
)
