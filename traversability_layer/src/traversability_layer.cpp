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
 * traversability_layer.cpp
 * @brief Implementation of the traversability layer plugin
 * @author: Luca Bartolomei
 * Created on: June 05, 2020
 */

#include "traversability_layer/traversability_layer.h"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(traversability_layer::TraversabilityLayer,
                       costmap_2d::Layer)

namespace traversability_layer
{

TraversabilityLayer::TraversabilityLayer() { dsrv_ = NULL; }

TraversabilityLayer::~TraversabilityLayer()
{
  if (dsrv_)
  {
    delete dsrv_;
  }
}

void TraversabilityLayer::onInitialize()
{
  // Generate node handle and initialize map size
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  // Update auxiliaries
  last_reading_time_ = ros::Time::now();
  default_value_ = to_cost(0.5);

  matchSize();
  min_x_ = min_y_ = -std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::max();
  buffered_maps_ = 0;
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Read the parameters from ROS server
  std::string topic_ns, topic_name;
  nh.param("ns", topic_ns, std::string());
  nh.param("topic", topic_name, std::string());

  nh.param("no_readings_timeout", no_readings_timeout_, 1.0);
  nh.param("traversability_layer", trav_layer_, std::string("traversability"));

  nh.param("traversable_threshold", traversable_threshold_, 0.6);
  nh.param("enabled", enabled_, false);
  nh.param("use_maximum", use_maximum_, false);

  // Check that we have an actual topic name
  if (topic_name.empty())
  {
    ROS_ERROR("[Traversability Layer] Topic name is empty!");
    return;
  }
  else
  {
    ROS_INFO_STREAM("[Traversability Layer] Topic name: " << topic_name);
    ROS_INFO_STREAM("[Traversability Layer] Topic ns  : " << topic_ns);
  }

  // Create the subscriber to the traversability map
  std::string topic(topic_ns + "/" + topic_name);
  ROS_INFO_STREAM("[Traversability Layer] Subscribed to: " << topic);
  trav_map_sub_ = nh.subscribe(
      topic, 100, &TraversabilityLayer::traversabilityMapCallback, this);

  // RQT dynamic reconfigure
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&TraversabilityLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  costmap_2d::GenericPluginConfig default_config;
  default_config.enabled = enabled_;
  dsrv_->setConfigDefault(default_config);
}

void TraversabilityLayer::updateBounds(double robot_x, double robot_y,
                                       double robot_yaw, double* min_x,
                                       double* min_y, double* max_x,
                                       double* max_y)
{
  if (layered_costmap_->isRolling())
    updateOrigin(robot_x - getSizeInMetersX() / 2,
                 robot_y - getSizeInMetersY() / 2);

  updateCostmap();

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  min_x_ = min_y_ = std::numeric_limits<double>::max();
  max_x_ = max_y_ = std::numeric_limits<double>::min();

  if (!enabled_)
  {
    current_ = true;
    return;
  }

  if (buffered_maps_ == 0)
  {

    if (no_readings_timeout_ > 0.0 &&
        (ros::Time::now() - last_reading_time_).toSec() > no_readings_timeout_)
    {
      ROS_WARN_THROTTLE(
          2.0,
          "[Traversability Layer] No traversability map received "
          "for %.2f seconds, while expected at least every %.2f seconds.",
          (ros::Time::now() - last_reading_time_).toSec(),
          no_readings_timeout_);
      current_ = false;
    }
  }
}

void TraversabilityLayer::traversabilityMapCallback(
    const grid_map_msgs::GridMap& trav_map_msg)
{
  boost::mutex::scoped_lock lock(trav_map_mutex_);
  ROS_INFO_ONCE("[Traversability Layer] Received first map");
  trav_msgs_buffer_.push_back(trav_map_msg);
}

void TraversabilityLayer::updateCostmap()
{
  if (trav_msgs_buffer_.empty())
  {
    return;
  }

  // Copy the values in the buffer first (mutex-protected)
  std::list<grid_map_msgs::GridMap> trav_msgs_buffer_copy;

  trav_map_mutex_.lock();
  trav_msgs_buffer_copy = std::list<grid_map_msgs::GridMap>(trav_msgs_buffer_);
  trav_msgs_buffer_.clear();
  trav_map_mutex_.unlock();

  for (std::list<grid_map_msgs::GridMap>::iterator trav_msgs_it =
           trav_msgs_buffer_copy.begin();
       trav_msgs_it != trav_msgs_buffer_copy.end(); trav_msgs_it++)
  {
    processTraversabilityMap(*trav_msgs_it);
  }
}

void TraversabilityLayer::processTraversabilityMap(
    const grid_map_msgs::GridMap& trav_map_msg)
{
  // Check that the reference frames match
  if (trav_map_msg.info.header.frame_id != global_frame_)
  {
    ROS_DEBUG("[Traversability Layer] Traversability map reference frame "
              "and costmap frame do not match!");
  }

  // Get the costmap to update
  costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();

  // Get the transformation between the traversability map and the cost map
  tf::StampedTransform transform;
  try
  {
    listener.lookupTransform(trav_map_msg.info.header.frame_id, global_frame_,
                             ros::Time(0), transform);
  }
  catch (const tf::TransformException & ex)
  {
    ROS_ERROR_THROTTLE(5.0, "[Traversability Layer] %s", ex.what());
    return;
  }

  // Convert the grid map ROS message to a grid map
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromMessage(trav_map_msg, grid_map);

  // Check the resolutions of the grid map and of the costmap
  const double resolution_map = grid_map.getResolution();
  const double resolution_costmap = master->getResolution();
  if (resolution_map != resolution_costmap)
  {
    ROS_DEBUG_STREAM("[Traversability Layer] Resolutions do not match.\n"
                     "\tTraversability map resolution: "
                     << resolution_map
                     << "\n\tCostmap resolution: " << resolution_costmap);
  }

  // Check that the traversability layer exits
  if (std::find(grid_map.getLayers().begin(), grid_map.getLayers().end(),
                trav_layer_) == grid_map.getLayers().end())
  {
    ROS_ERROR_STREAM("[Traversability Layer] Layer "
                     << trav_layer_ << " does not exist in the map!");
    return;
  }

  // Iterate over the cells of the grid map. Here we get the cell coordinates,
  // convert in the costmap coordinates and update the value
  grid_map::Matrix& trav_data = grid_map[trav_layer_];

  for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd();
       ++iterator)
  {
    const int i = iterator.getLinearIndex();

    // If the traversability value is invalid, just move to the next value
    if (std::isnan(trav_data(i)))
    {
      continue;
    }

    // If the traversability value is valid, then we need to update the value
    // in the costmap as well
    // First, get the (x,y) position from the grid map
    Eigen::Vector2d position;
    grid_map.getPosition(iterator.getUnwrappedIndex(), position);

    tf::Point p(position.x(), position.y(), 0);
    p = transform(p);

    // Get the costmap indices values from (x,y) position
    unsigned int map_x, map_y;
    if (!master->worldToMap(p.x(), p.y(), map_x, map_y))
    {
      ROS_ERROR("[Traversability Layer] Could not covert to map coordinates");
      continue;
    }

    // Get the cost and update the corresponding cell
    unsigned char cost = to_cost(trav_data(i));
    if (!use_maximum_)
    {
      master->setCost(map_x, map_y, cost);
    }
    else
    {
      master->setCost(map_x, map_y,
                      std::max(cost, master->getCost(map_x, map_y)));
    }
  }

  // Update auxiliaries
  buffered_maps_++;
  last_reading_time_ = ros::Time::now();
}

void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                      int min_i, int min_j, int max_i,
                                      int max_j)
{
  // TODO(lucaBartolomei) : Need to add intermediate traversability score?
  if (!enabled_)
  {
    return;
  }

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  unsigned char traversable = to_cost(traversable_threshold_);

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      unsigned char cost = costmap_[it];
      unsigned char current;
      if (cost == costmap_2d::NO_INFORMATION)
      {
        it++;
        continue;
      }
      else if (cost >= traversable)
      {
        current = costmap_2d::FREE_SPACE;
      }
      else if (cost < traversable)
      {
        current = costmap_2d::LETHAL_OBSTACLE;
      }
      else
      {
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];

      if (old_cost == costmap_2d::NO_INFORMATION || old_cost < current)
      {
        master_array[it] = current;
      }
      it++;
    }
  }

  buffered_maps_ = 0;
  current_ = true;
}

void TraversabilityLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config,
                                        uint32_t level)
{
  // TODO : fill this in with the other parameters
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
  }
}

void TraversabilityLayer::reset()
{
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

void TraversabilityLayer::deactivate() { trav_msgs_buffer_.clear(); }

void TraversabilityLayer::activate() { trav_msgs_buffer_.clear(); }

} // end namespace traversability_layer
