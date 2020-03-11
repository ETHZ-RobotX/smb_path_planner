cmake_minimum_required(VERSION 2.8.3)
project(smb_navigation)

add_compile_options(-std=c++11 -Wall)
add_definitions(-DMELO_FUNCTION_PRINTS)

set(PACKAGE_DEPENDENCIES 
        roscpp
        move_base
        teb_local_planner
        pointcloud_to_laserscan
        robot_self_filter
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
