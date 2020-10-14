/*
 * Copyright (c) 2020, Vision for Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * traversability_layer.h
 * @brief Header of the traversability layer class
 * @author: Luca Bartolomei
 * Created on: June 05, 2020
 */

#pragma once

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace traversability_layer
{

class TraversabilityLayer : public costmap_2d::CostmapLayer
{

public:
  /**
   * @brief Class constructor
   */
  TraversabilityLayer();

  ~TraversabilityLayer();

  /**
   * @brief Method that start the layer and create the necessary communication
   */
  virtual void onInitialize();

  /**
   * @brief Method to update the costmap bounds
   * @param[in] robot_x : robot X position
   * @param[in] robot_y : robot Y position
   * @param[in] robot_yaw : robot Yaw position
   * @param[out] min_x : minimum X map
   * @param[out] min_y : minimum Y map
   * @param[out] max_x : maximum X map
   * @param[out] max_y : maximum Y map
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);

  /**
   * @brief Method that updates the overall costmap with the info from the
   *        traversability_layer
   * @param[out] master_grid : map to be updated
   * @param[in] min_i : min row index
   * @param[in] min_j : min col index
   * @param[in] max_i : max row index
   * @param[in] max_j : max col index
   */
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);

  /**
   * @brief Utility map management
   */
  virtual void reset();
  virtual void deactivate();
  virtual void activate();

private:
  /**
   * @brief Callback receiving a traversability map as input
   * @param[in] trav_map_msg : incoming message
   */
  void traversabilityMapCallback(const grid_map_msgs::GridMap& trav_map_msg);

  /**
   * @brief Method to update the cost map from the traversability map buffer
   */
  void updateCostmap();

  /**
   * @brief Method to process a single grid map message to be transformed in
   *        a costmap_2d
   * @param[in] trav_map_msg : message to be processed
   */
  void processTraversabilityMap(const grid_map_msgs::GridMap& trav_map_msg);

  /**
   * @brief Method that transforms the traversability value (in range [0,1])
   *        into a cost value (char in [0,255])
   * @param[in] t : input traversability value in [0,1] = [min_trav, max_trav]
   * @return cost in costmap_2d format
   */
  unsigned char to_cost(double t)
  {
    return static_cast<unsigned char>((1.0 - t) * costmap_2d::LETHAL_OBSTACLE);
  }

  /**
   * @brief Callback for dynamic reconfigure
   * @param[out] config : configuration to be changed
   * @param[in] level
   */
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);

protected:
  // ROS-related member class
  ros::Subscriber trav_map_sub_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
  tf::TransformListener listener;

  // Map management
  std::list<grid_map_msgs::GridMap> trav_msgs_buffer_;

  ros::Time last_reading_time_;
  double no_readings_timeout_;

  double min_x_, min_y_, max_x_, max_y_;
  double traversable_threshold_;
  bool use_maximum_;

  unsigned int buffered_maps_;
  std::string global_frame_;
  std::string trav_layer_;

  // Auxiliaries
  boost::mutex trav_map_mutex_;

}; // end class TraversabilityLayer

} // end namespace traversability_layer
