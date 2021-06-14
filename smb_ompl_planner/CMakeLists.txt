cmake_minimum_required(VERSION 2.8.3)
project(smb_ompl_planner)
set(CMAKE_BUILD_TYPE Release)

add_compile_options(-std=c++11 -Wall)

# Find packages
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(catkin REQUIRED
    COMPONENTS
        costmap_2d
        geometry_msgs
        nav_core
        navfn
        nav_msgs
        pluginlib
        roscpp
        tf2_geometry_msgs
        tf2_ros
)

catkin_package(
    INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        nav_core
        navfn
        nav_msgs
        pluginlib
        roscpp
        tf2_ros
)

# Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ompl_planner.cpp
  src/rrt_planner.cpp
  src/orientation_filter.cpp
)
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

# Executable
add_executable(${PROJECT_NAME}_node
    src/ompl_planner_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

