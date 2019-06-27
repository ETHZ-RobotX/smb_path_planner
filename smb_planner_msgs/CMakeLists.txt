cmake_minimum_required(VERSION 2.8.3)
project(smb_planner_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs message_generation)

add_service_files(
  FILES
  PlannerService.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)
