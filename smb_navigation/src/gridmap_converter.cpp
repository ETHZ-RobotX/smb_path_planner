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
 * gridmap_converter.cpp
 * @brief Implementation of the gridmap converter class
 * @author: Luca Bartolomei
 * Created on: June 04, 2020
 *
 * This software takes inspiration from the work done by
 * - Bossard Anna, ETH Zurich, Rowesys Team
 * - Lieberherr Pascal, ETH Zurich, Rowesys Team
 * More info at https://rowesys.ethz.ch/
 */

#include "smb_navigation/gridmap_converter.h"

#include <grid_map_ros/GridMapRosConverter.hpp>

namespace smb_navigation
{

GridMapConverter::GridMapConverter(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), initialized_(false)
{
  // Make sure we have the parameters
  initialized_ = readParameters();

  // Initialize subscriber
  traversability_map_sub_ =
      nh_.subscribe(traversability_map_topic_, 10,
                    &GridMapConverter::traversabilityMapCallback, this);

  // Initialize publisher
  occupancy_map_pub_ =
      nh_.advertise<nav_msgs::OccupancyGrid>(occupancy_map_topic_, 10);
}

void GridMapConverter::traversabilityMapCallback(
    const grid_map_msgs::GridMap& grid_map_msg)
{
  // Convert the grid map ROS message to a grid map
  grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map_);

  // Convert from grid map to costmap_2d
  nav_msgs::OccupancyGrid occupancy_map_msg;
  try
  {
    grid_map::GridMapRosConverter::toOccupancyGrid(
        grid_map_, trav_layer_, min_value_grid_map_, max_value_grid_map_,
        occupancy_map_msg);

    // The traversability map is in [0,1] where 0 is untraversable and 1 fully
    // traversable. Since in move_base the higher the value, the higher the cost
    // we need to "invert" the traversability scale (i.e. 0: no traversability
    // cost, 1: maximum traversability cost)
    int OccSize = occupancy_map_msg.info.width * occupancy_map_msg.info.height;

    for (int i = 0; i < OccSize; ++i)
    {
      if (occupancy_map_msg.data[i] != -1)
      {
        int dataTemp = occupancy_map_msg.data[i];
        occupancy_map_msg.data[i] = 100 - dataTemp;
      }
    }
  }
  catch (std::out_of_range::exception&)
  {
    ROS_ERROR_STREAM("[GridMap Converter] Layer " << trav_layer_
                                                  << " does not exist!");
    return;
  }

  ROS_INFO_ONCE("[GridMap Converter] Published first occupancy map");
  occupancy_map_pub_.publish(occupancy_map_msg);
}

bool GridMapConverter::readParameters()
{
  if (!nh_private_.getParam("gridmap_converter/layer", trav_layer_))
  {
    ROS_ERROR("[GridMap Converter] Output layer not specified");
    return false;
  }

  if (!nh_private_.getParam("gridmap_converter/max_value_grid_map",
                            max_value_grid_map_))
  {
    ROS_ERROR("[GridMap Converter] Max value grid map not specified");
    return false;
  }

  if (!nh_private_.getParam("gridmap_converter/min_value_grid_map",
                            min_value_grid_map_))
  {
    ROS_ERROR("[GridMap Converter] Min value grid map not specified");
    return false;
  }

  if (!nh_private_.getParam("gridmap_converter/traversability_topic",
                            traversability_map_topic_))
  {
    ROS_ERROR("[GridMap Converter] Traversability topic not specified");
    return false;
  }

  if (!nh_private_.getParam("gridmap_converter/occupancy_map_topic",
                            occupancy_map_topic_))
  {
    ROS_ERROR("[GridMap Converter] Occupancy map topic not specified");
    return false;
  }

  ROS_INFO("[GridMap Converter] Parameters parsed correctly");
  return true;
}

} // end namespace smb_navigation
