cmake_minimum_required(VERSION 2.8.3)
project(smb_planner_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
    src/utility_mapping.cpp
    src/utility_visualization.cpp
    src/traversability_estimator.cpp
    src/planner_parameters.cpp
)

cs_add_executable(smb_pose_interceptor_node
		src/smb_pose_interceptor.cpp
)
target_link_libraries(smb_pose_interceptor_node ${CATKIN_LIBRARIES})

cs_add_executable(smb_tf_interceptor_node
		src/smb_tf_interceptor.cpp
)
target_link_libraries(smb_tf_interceptor_node ${CATKIN_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
