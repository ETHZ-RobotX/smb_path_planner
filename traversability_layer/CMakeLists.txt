cmake_minimum_required(VERSION 2.8.3)
project(traversability_layer)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

add_compile_options(-std=c++11 -Wall)

set(PACKAGE_DEPENDENCIES
    roscpp
    pluginlib
    costmap_2d
    grid_map_costmap_2d
    grid_map_ros
)

find_package(catkin REQUIRED COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

# Build
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/traversability_layer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

# Testing
if(CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS roslint)
    roslint_cpp()
    roslint_add_test()
endif()

# Install
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
