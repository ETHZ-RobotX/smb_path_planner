/**
  * @author Luca Bartolomei, V4RL
  * @brief  Utility file that contains all useful methods visualization.
  * @date   13.06.2019
  */

#pragma once

#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#include "smb_planner_common/utility_mapping.h"

namespace smb_planner {
namespace utility_visualization {

    /**
     * @brief Class to quickly decide the color to use
     */
    class Color : public std_msgs::ColorRGBA {
    public:
        Color() : std_msgs::ColorRGBA() {}
        Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
        Color(double red, double green, double blue, double alpha) : Color() {
            r = red;
            g = green;
            b = blue;
            a = alpha;
        }

        static const Color White() { return Color(1.0, 1.0, 1.0); }
        static const Color Black() { return Color(0.0, 0.0, 0.0); }
        static const Color Gray() { return Color(0.5, 0.5, 0.5); }
        static const Color Red() { return Color(1.0, 0.0, 0.0); }
        static const Color Green() { return Color(0.0, 1.0, 0.0); }
        static const Color Blue() { return Color(0.0, 0.0, 1.0); }
        static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
        static const Color Orange() { return Color(1.0, 0.5, 0.0); }
        static const Color Purple() { return Color(0.5, 0.0, 1.0); }
        static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
        static const Color Teal() { return Color(0.0, 1.0, 1.0); }
        static const Color Pink() { return Color(1.0, 0.0, 0.5); }
    };

    /**
     * @brief Method to create a marker containing the whole planned path as
     *        a line strip using traversability information
     * @param[in] path : path to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the line strip (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @param[in] traversability_map : traversability map to project path
     *         according to elevation and traversability maps
     * @param[in] traversability_threshold : threshold to consider a point
     *         traversable
     * @param[in] maximum_difference_elevation : threshold to consider a
	 *             difference in the height acceptable
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createMarkerForPathWithTraversability(
            const std::vector<Eigen::Vector3d>& path,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id,
            const double planning_height,
            const grid_map::GridMap& traversability_map,
            const double traversability_threshold,
            const double maximum_difference_elevation);

    /**
     * @brief Method to create a marker containing the whole planned path as
     *        a line strip
     * @param[in] path : path to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the line strip (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createMarkerForPath(
            const std::vector<Eigen::Vector3d>& path,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id,
            const double planning_height);

    /**
     * @brief Method to create a marker containing the whole planned path as
     *        a sphere list using traversability information
     * @param[in] path : path to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the spheres (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @param[in] traversability_map : traversability map to project path
     *         according to elevation and traversability maps
     * @param[in] traversability_threshold : threshold to consider a point
     *         traversable
     * @param[in] maximum_difference_elevation : threshold to consider a
	 *             difference in the height acceptable
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createMarkerForWaypointsWithTraversability(
            const std::vector<Eigen::Vector3d>& path,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id,
            const double planning_height,
            const grid_map::GridMap& traversability_map,
            const double traversability_threshold,
            const double maximum_difference_elevation);

    /**
     * @brief Method to create a marker containing the whole planned path as
     *        a sphere list
     * @param[in] path : path to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the spheres (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createMarkerForWaypoints(
            const std::vector<Eigen::Vector3d>& path,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id,
            const double planning_height);

    /**
     * @brief Method to create a marker containing a state of the whole planned
     *        path as a sphere using traversability information
     * @param[in] state : state to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the sphere (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @param[in] traversability_map : traversability map to project path
     *         according to elevation and traversability maps
     * @param[in] traversability_threshold : threshold to consider a point
     *         traversable
     * @param[in] maximum_difference_elevation : threshold to consider a
	 *             difference in the height acceptable
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createMarkerForStateWithTraversability(
            const Eigen::Vector3d &state,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id,
            const double planning_height,
            const grid_map::GridMap& traversability_map,
            const double traversability_threshold,
            const double maximum_difference_elevation);

    /**
     * @brief Method to create a marker containing a state of the whole planned
     *        path as a sphere
     * @param[in] path : path to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the sphere (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createMarkerForState(
            const Eigen::Vector3d &state,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id);

    /**
     * @brief Method to create a marker containing a state of the whole planned
     *        path as a arrow to show orientation using traversability info
     * @param[in] state : state to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the arrows (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @param[in] traversability_map : traversability map to project path
     *         according to elevation and traversability maps
     * @param[in] traversability_threshold : threshold to consider a point
     *         traversable
     * @param[in] maximum_difference_elevation : threshold to consider a
	 *             difference in the height acceptable
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createArrowMarkerForStateWithTraversability(
            const Eigen::Vector3d& state,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id,
            const double planning_height,
            const grid_map::GridMap& traversability_map,
            const double traversability_threshold,
            const double maximum_difference_elevation);

    /**
     * @brief Method to create a marker containing a state of the whole planned
     *        path as a arrow to show orientation
     * @param[in] path : path to be visualized
     * @param[in] color : color of the path
     * @param[in] name : namespace for the marker in rviz
     * @param[in] scale : scale of the arrows (dimensions)
     * @param[in] frame_id : frame id for the marker
     * @param[in] planning_height : nominal planned height
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createArrowMarkerForState(
            const Eigen::Vector3d& state,
            const std_msgs::ColorRGBA& color,
            const std::string& name,
            const double scale,
            const std::string& frame_id,
            const double planning_height);

    /**
     * @brief Method to create a marker containing the sphere for collision
     *        checks using traversability info
     * @param[in] state : state to be visualized with bounding box
     * @param[in] name : namespace for the marker in rviz
     * @param[in] frame_id : frame id for the marker
     * @param[in] robot_radius : scale of the sphere (dimensions)
     * @param[in] planning_height : nominal planned height
     * @param[in] traversability_map : traversability map to project path
     *         according to elevation and traversability maps
     * @param[in] traversability_threshold : threshold to consider a point
     *         traversable
     * @param[in] maximum_difference_elevation : threshold to consider a
	 *             difference in the height acceptable
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createSphereCollisionForStateWithTraversability(
            const Eigen::Vector3d &state,
            const std::string& name,
            const std::string& frame_id,
            const double robot_radius,
            const double planning_height,
            const grid_map::GridMap& traversability_map,
            const double traversability_threshold,
            const double maximum_difference_elevation);

    /**
     * @brief Method to create a marker containing the sphere for collision
     *        checks using traversability info
     * @param[in] state : state to be visualized with bounding box
     * @param[in] name : namespace for the marker in rviz
     * @param[in] frame_id : frame id for the marker
     * @param[in] robot_radius : scale of the sphere (dimensions)
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createSphereCollisionForState(
            const Eigen::Vector3d &state,
            const std::string& name,
            const std::string& frame_id,
            const double robot_radius);
            
    /**
     * @brief Method to create a marker to show to boundaries for planning
     * @param[in] lower : lower bound (x,y,z)
     * @param[in] upper : upper bound (x,y,z)
     * @param[in] name : namespace for the marker in rviz
     * @param[in] frame_id : frame id for the marker
     * @return marker to be visualized in rviz
     */
    visualization_msgs::Marker createBoundaries(
            const Eigen::Vector3d &lower, 
            const Eigen::Vector3d &upper,
            const std::string &name, 
            const std::string &frame_id);

} // end namespace utility_visualization
} // end namespace smb_planner
