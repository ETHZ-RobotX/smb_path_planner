cmake_minimum_required(VERSION 2.8.3)
project(smb_navigation)
set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-std=c++11 -Wall)

set(PACKAGE_DEPENDENCIES
        roscpp
        move_base
        teb_local_planner
        pointcloud_to_laserscan
        robot_self_filter
        smb_ompl_planner
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

add_library(path_interceptor
    src/path_interceptor.cpp
)

target_link_libraries(path_interceptor
    ${catkin_LIBRARIES})

add_dependencies(path_interceptor
    ${catkin_EXPORTED_TARGETS})

add_executable(path_interceptor_node
    src/path_interceptor_node.cpp)

add_dependencies(path_interceptor_node
    path_interceptor
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_interceptor_node
    path_interceptor
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)

install(TARGETS path_interceptor path_interceptor_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

