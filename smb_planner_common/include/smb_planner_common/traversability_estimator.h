/**
  * @author Luca Bartolomei, V4RL
  * @brief  Traversability estimator that converts an elevation map to a
  *         traversability map.
  * @date   13.06.2019
  */

#pragma once

#include <yaml-cpp/yaml.h>
#include <traversability_estimation/TraversabilityMap.hpp>

namespace smb_planner {

    class TraversabilityEstimator {
    public:

        /**
         * @brief Class constructor
         * @param[in] nh : ros node handle for ros communication
         */
        TraversabilityEstimator(const ros::NodeHandle& nh);

        /**
         * @brief Class destructor
         */
        ~TraversabilityEstimator() {}

        /**
         * @brief Method to get the traversability map from object
         * @return Traversability map as pointer
         */
        std::shared_ptr<traversability_estimation::TraversabilityMap>
                getTraversabilityMap() {
            return traversability_map_;
        }

        /**
         * @brief Method to get the traversability map from object
         * @return Traversability map as a grid_map message
         */
        grid_map::GridMap getGridMapTraversability() {
            return traversability_map_->getTraversabilityMap();
        }
        
        bool hasElevationMap() const { return got_elevation_map_; }

    private:
        /**
         * @brief Callback to process the elevation map into traversability map
         * @param[in] msg : Elevation map message
         */
        void elevationMapCallback(const grid_map_msgs::GridMap& msg);

        /**
         * @brief Actual method that converts elevation map into traversability
         * @param[in] gridMap : Elevation Map
         * @return True if conversion is successful, false otherwise
         */
        bool setupTraversabilityMap(const grid_map::GridMap& gridMap);

    protected:
        // ROS variables to listen to the traversability map topic
        ros::NodeHandle nh_;
        ros::Subscriber elevation_map_sub_;

        // Traversability map
        std::vector<std::string> elevation_map_layers_;
        std::shared_ptr<traversability_estimation::TraversabilityMap>
                traversability_map_;
                
        // Check variable
        bool got_elevation_map_;
    };

}  // namespace smb_planner
