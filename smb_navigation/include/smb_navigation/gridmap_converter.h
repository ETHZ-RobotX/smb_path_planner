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
 * gridmap_converter.h
 * @brief Header of the gridmap converter class
 * @author: Luca Bartolomei
 * Created on: June 04, 2020
 *
 * This software takes inspiration from the work done by
 * - Bossard Anna, ETH Zurich, Rowesys Team
 * - Lieberherr Pascal, ETH Zurich, Rowesys Team
 * More info at https://rowesys.ethz.ch/
 */

#include <costmap_2d/costmap_2d_publisher.h>
#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>

namespace smb_navigation
{

class GridMapConverter
{

public:
  GridMapConverter(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

  ~GridMapConverter() {}

  bool isInitialized() const { return initialized_; }

private:
  // Callbacks

  /**
   * @brief Callback from traversability estimation package
   * @param[in] gridmap_msg : input grid map message
   */
  void traversabilityMapCallback(const grid_map_msgs::GridMap& grid_map_msg);

  // Auxiliaries

  /**
   * @brief Function to read parameters from server
   * @return True if all parameters have been found, False otherwise
   */
  bool readParameters();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber traversability_map_sub_;
  ros::Publisher occupancy_map_pub_;
  
  grid_map::GridMap grid_map_;
  float max_value_grid_map_;
  float min_value_grid_map_;

  std::string traversability_map_topic_;
  std::string occupancy_map_topic_;
  std::string trav_layer_;

  bool initialized_;

}; // end class GridMapConverter

} // end namespace smb_navigation
