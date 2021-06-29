cmake_minimum_required(VERSION 2.8.3)
project(smb_navigation)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_compile_options(-std=c++14 -Wall)

# Set dependencies
set(PACKAGE_DEPENDENCIES
    roscpp
    move_base
    grid_map_costmap_2d
    grid_map_ros
    pcl_ros
    octomap_ros
)

find_package(catkin REQUIRED COMPONENTS 
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS
)

# Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

### Path interceptor
add_library(path_interceptor
    src/path_interceptor.cpp)

target_link_libraries(path_interceptor
    ${catkin_LIBRARIES})

add_dependencies(path_interceptor
    ${catkin_EXPORTED_TARGETS})

add_executable(path_interceptor_node
    src/nodes/path_interceptor_node.cpp)

add_dependencies(path_interceptor_node
    path_interceptor
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_interceptor_node
    path_interceptor)

### Grid Map converter
add_library(gridmap_converter
    src/gridmap_converter.cpp
)

target_link_libraries(gridmap_converter
    ${catkin_LIBRARIES})

add_dependencies(gridmap_converter
    ${catkin_EXPORTED_TARGETS})

add_executable(gridmap_converter_node
    src/nodes/gridmap_converter_node.cpp)

add_dependencies(gridmap_converter_node
    gridmap_converter
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(gridmap_converter_node
    gridmap_converter)

### PCD converter
add_library(pcd_converter
    src/pcd_converter.cpp
)

target_link_libraries(pcd_converter
    ${catkin_LIBRARIES})

add_dependencies(pcd_converter
    ${catkin_EXPORTED_TARGETS})

add_executable(pcd_converter_node
    src/nodes/pcd_converter_node.cpp)

add_dependencies(gridmap_converter_node
    gridmap_converter
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(pcd_converter_node
    pcd_converter)

### Odometry interceptor
add_executable(odometry_interceptor_node
    src/nodes/odom_interceptor_node.cpp)

add_dependencies(odometry_interceptor_node
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(odometry_interceptor_node
    ${catkin_LIBRARIES})

### Pose Stamped interceptor
add_executable(pose_interceptor_node
    src/nodes/pose_interceptor_node.cpp)

add_dependencies(pose_interceptor_node
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_interceptor_node
    ${catkin_LIBRARIES})

### Install
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)

install(TARGETS path_interceptor
                path_interceptor_node
                gridmap_converter
                gridmap_converter_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

