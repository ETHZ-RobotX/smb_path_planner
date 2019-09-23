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
         * @param[in] n_sensors : number of sensors used for traversability est
         * @param[in] maps_weights : weights for the different elevation maps
         */
        TraversabilityEstimator(const ros::NodeHandle& nh,
                                const std::vector<double> &maps_weights,
                                const int n_sensors = 1);

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
        
        bool hasElevationMap() const { return got_elevation_map_[0]; }
        bool hasElevationMap(const int n_sensor) const {
          return got_elevation_map_[n_sensor];
        }

    private:
        /**
         * @brief Callback to process the elevation map into traversability map
         * @param[in] msg : Elevation map message
         */
        void elevationMapCallback(const grid_map_msgs::GridMap& msg);
        
        /**
         * @brief Callback to process the elevation map into traversability map
         * @param[in] msg : Elevation map message
         * @param[in] n_sensor : ID of the sensor
         */
        void elevationSensorMapCallback(
               const grid_map_msgs::GridMapConstPtr &msg,
               const int n_sensor);

        /**
         * @brief Actual method that converts elevation map into traversability
         * @param[in] gridMap : Elevation Map
         * @return True if conversion is successful, false otherwise
         */
        bool setupTraversabilityMap(const grid_map::GridMap& gridMap);

        /**
         * @brief Method to get a fused elevation map from the ones stored in
         *        memory, and then compute the traversability map out of it
         * @return True if traversability extraction was successfull, False oth.
         */
        bool setupFusedTraversabilityMap();

    protected:
        // ROS variables to listen to the traversability map topic
        ros::NodeHandle nh_;
        std::vector<ros::Subscriber> elevation_map_subs_;

        // Traversability map
        int n_sensors_;
        std::vector<double> weights_;
        double weights_sum_;

        std::vector<grid_map::GridMap> elevation_grid_maps_;
        std::vector<std::string> elevation_map_layers_;
        std::shared_ptr<traversability_estimation::TraversabilityMap>
                traversability_map_;
                
        // Check variable
        std::vector<bool> got_elevation_map_;
    };

}  // namespace smb_planner
