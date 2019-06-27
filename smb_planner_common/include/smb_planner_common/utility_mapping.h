/**
  * @author Luca Bartolomei, V4RL
  * @brief  Utility file that contains all useful methods for traversability
  *         checks and information extraction from voxblox.
  * @date   13.06.2019
  */

#pragma once

#include <math.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "smb_planner_common/planner_parameters.h"

namespace smb_planner {
namespace utility_mapping {

    /**
     * @brief Method that gets the normal vector to the ground in local map
     * @param[in] map : local traversability map
     * @param[in] normal_index_x : index for the normal along X in map structure
     * @param[in] normal_index_y : index for the normal along Y in map structure
     * @param[in] normal_index_z : index for the normal along Z in map structure
     * @param[out] n : normal to the ground
     * @return True if the position has been observed, False if not
     */
	bool getNormalVectorFromGridMap(
			const grid_map_msgs::GridMap& map,
			const int normal_index_x,
			const int normal_index_y,
			const int normal_index_z,
			Eigen::Vector3d& n);

	/**
	 * @brief Method to get the traversability in a local map
	 * @param[in] map : local traversability map
	 * @param[in] traversability_index : index for the traversability
	 *             in map structure
	 * @param[out] traversability_val : traversability value in local map
	 * @return True if the position has been observed, False if not
	 */
	bool getTraversabilityValueFromGridMap(
			const grid_map_msgs::GridMap& map,
			const int traversability_index,
			double* traversability_val);

	/**
	 * @brief Method to get the elevation in a local map
	 * @param[in] map : local traversability map
	 * @param[in] elevation_index : index for the elevation in map structure
	 * @param[out] elevation : elevation value in local map
	 * @return True if the position has been observed, False if not
	 */
	bool getElevationValueFromGridMap(
			const grid_map_msgs::GridMap& map,
			const int elevation_index,
			double* elevation);

	/**
	 * @brief Method to get the projected position in (X,Y,Z) on the
	 *        traversability map for a query position
	 * @param[in] traversability_map : full traversability map
	 * @param[in] position : query position to be projected
	 * @param[in] projection_distance : nominal height on the terrain
	 * @param[in] traversability_threshold : threshold to consider if the
     *             position is traversable
	 * @param[in] maximum_difference_elevation : threshold to consider a
	 *             difference in the height acceptable
	 * @param[out] projection : projected position in the traversability map
	 * @return Traversability status of the query position (TRAVERSABLE,
	 *          UNTRAVERSABLE, UNKNOWN)
	 */
	TraversabilityStatus getTrasversabilityInformation(
			const grid_map::GridMap& traversability_map,
			const Eigen::Vector2d& position,
			const double projection_distance,
			const double traversability_threshold,
			const double maximum_difference_elevation,
			Eigen::Vector3d& projection);

	/**
	 * @brief Method to check the traversability of query positions and to
	 *        project the query positions in the traversability map
	 * @param[in] traversability_map : full traversability map
	 * @param[in] positions : query positions to be projected
	 * @param[in] projection_distance : nominal height on the terrain
	 * @param[in] traversability_threshold : threshold to consider if the
     *             position is traversable
	 * @param[in] maximum_difference_elevation : threshold to consider a
	 *             difference in the height acceptable
	 * @param projections  : projected positions in the traversability map
	 * @param status : Traversability status of the query positions
	 *          (TRAVERSABLE, UNTRAVERSABLE, UNKNOWN)
	 * @return False if one of the query positions is untraversable, true oth.
	 */
	bool checkTraversabilityMultiplePositions(
			const grid_map::GridMap& traversability_map,
			const std::vector<Eigen::Vector2d>& positions,
			const double projection_distance,
			const double traversability_threshold,
			const double maximum_difference_elevation,
			std::vector<Eigen::Vector3d>& projections,
			std::vector<TraversabilityStatus>& status);

    /**
     * @brief Method to check the traversability of a query position and to
     *        project the query position in the traversability map
     * @param[in] traversability_map : full traversability map
     * @param[in] position : query position to be projected
     * @param[in] projection_distance : nominal height on the terrain
     * @param[in] traversability_threshold : threshold to consider if the
     *             position is traversable
     * @param[in] maximum_difference_elevation : threshold to consider a
     *             difference in the height acceptable
     * @param projection  : projected position in the traversability map
     * @return False if one of the query positions is untraversable, true oth.
     */
	bool checkTraversabilityPosition(
			const grid_map::GridMap& traversability_map,
			const Eigen::Vector2d& position,
			const double projection_distance,
			const double traversability_threshold,
			const double maximum_difference_elevation,
			Eigen::Vector3d& projection);

	/**
	 * @brief Method to get the voxels around a query position in a given radius
	 * @tparam[in] VoxelType : ESDF or TSDF voxel type
	 * @param[in] layer : layer in voxblox to extract the info from
	 * @param[in] center : query position
	 * @param[in] radius : radius around query position to check
	 * @param[out] block_voxel_list : list of voxels in the query region
	 */
	void getSphereAroundPoint(
			const voxblox::Layer<voxblox::TsdfVoxel>& layer,
			const voxblox::Point& center,
			voxblox::FloatingPoint radius,
			voxblox::HierarchicalIndexMap* block_voxel_list);

	/**
	 * @brief Method that interpolates a list of waypoints linearly
	 * @param[in] waypoints : Waypoints to be interpolated
	 * @param[out] interpolated_waypoints : Output interpolated waypoints
	 * @param[in] v_max : maximum velocity between waypoints
	 * @param[in] a_max : maximum acceleration between waypoints
	 * @param[in] sampling_dt : sampling time for trajectory generation
	 * @return True if interpolation is successful, False otherwise
	 */
    bool rampInterpolatorWaypoints(
            const std::vector<Eigen::Vector3d>& waypoints,
            std::vector<Eigen::VectorXd>& interpolated_waypoints,
            const double v_max,
            const double a_max,
            const double sampling_dt);
    
    /**
     * @brief Method that get a vector of yaws from a sequence of 2D states. The
     *        yaws point from one state to the consecutive one.
     * @param[in] states : sequence of states to extract the yaws from
     * @param[out] yaws : extracted yaws from the path
     */        
    void createYawsFromStates(
            const std::vector<Eigen::Vector2d>& states,
            std::vector<double>& yaws);
            
    /**
     * @brief Method that normalizes an angle in the range [-pi, pi]
     * @return The normalized angle
     */        
    double pi2pi(const double angle);

} // end namespace utility_mapping
} // end namespace smb_planner
