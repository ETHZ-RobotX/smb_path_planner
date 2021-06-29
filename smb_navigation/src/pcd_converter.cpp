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
 * pcd_coverter.cpp
 * @brief Class that reads a PCD and outputs a point cloud pruned of the ground
 *        and of outliers points. This point cloud is store in an octomap
 * @author: Luca Bartolomei
 * Created on: June 08, 2021
 */

#include "smb_navigation/pcd_converter.h"

// std
#include <chrono>

// pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl_conversions/pcl_conversions.h>

// octomap
#include <octomap/octomap.h>

// ROS
#include <sensor_msgs/PointCloud2.h>

namespace smb_navigation
{

PcdConverter::PcdConverter(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), initialized_(false)
{
  // Make sure we have the parameters
  initialized_ = readParameters();

  // ROS
  pruned_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pruned_pcl", 1);
  original_pcl_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("pcl_original", 1);
}

void PcdConverter::generateMap()
{
  // Read the file in PCL format
  auto start_time = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile<pcl::PointXYZ>(p_input_file_, *input_cloud);

  auto end_time = std::chrono::high_resolution_clock::now();
  double time_read = std::chrono::duration_cast<std::chrono::milliseconds>(
                         end_time - start_time)
                         .count();
  ROS_INFO_STREAM(
      "[PCD Converter] Time to read input point cloud: " << time_read << " ms");

  // Inform the user about the size of the point cloud
  const auto pcl_size = input_cloud->points.size();
  ROS_INFO_STREAM("[PCD Converter] Input point cloud original size: "
                  << pcl_size << " points");

  // Downsample point cloud
  start_time = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(input_cloud);
  voxelGrid.setLeafSize(p_resolution_, p_resolution_, p_resolution_);
  voxelGrid.filter(*downsampled_cloud);

  end_time = std::chrono::high_resolution_clock::now();
  double time_downsample =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                            start_time)
          .count();
  ROS_INFO_STREAM("[PCD Converter] Time to downsample input point cloud: "
                  << time_downsample << " ms");

  // Inform the user about the size of the point cloud
  const auto downsampled_pcl_size = downsampled_cloud->points.size();
  ROS_INFO_STREAM("[PCD Converter] Size downsampled point cloud: "
                  << downsampled_pcl_size << " points");

  // Rotate the point cloud
  Eigen::Affine3f pcl_transform(Eigen::Affine3f::Identity());
  pcl_transform.rotate(
      Eigen::AngleAxisf(p_rot_roll_ * M_PI / 180, Eigen::Vector3f::UnitX()));
  pcl_transform.rotate(
      Eigen::AngleAxisf(p_rot_pitch_ * M_PI / 180, Eigen::Vector3f::UnitY()));
  pcl_transform.rotate(
      Eigen::AngleAxisf(p_rot_yaw_ * M_PI / 180, Eigen::Vector3f::UnitZ()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*downsampled_cloud, *rotated_cloud, pcl_transform);

  // Store the downsampled point cloud as ROS message (published later)
  sensor_msgs::PointCloud2 pcl_msg_old;
  pcl::toROSMsg(*rotated_cloud, pcl_msg_old);
  pcl_msg_old.header.frame_id = p_frame_id_;
  
  // Publish the original point cloud here as well 
  double time_pub_original = 0.0; // [s]
  while (ros::ok() && time_pub_original < 5.0)
  {
    //
    original_pcl_pub_.publish(pcl_msg_old);
    //
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    time_pub_original += 0.5;
  }

  // Filter the ground from the point cloud
  // Notice: this is the most expensive step. For a ~500 000 points it takes
  //         around 7 minutes
  ROS_WARN(
      "[PCD Converter] Removing the ground -- this can take several minutes!"
      " It depends on map resolution");
  start_time = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud(rotated_cloud);
  pmf.setMaxWindowSize(20);
  pmf.setSlope(1.0f);
  pmf.setInitialDistance(0.5f);
  pmf.setMaxDistance(2.0f);

  pcl::PointIndicesPtr ground(new pcl::PointIndices);
  pmf.extract(ground->indices);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(rotated_cloud);
  extract.setNegative(true);
  extract.setIndices(ground);
  extract.filter(*cloud_filtered);

  end_time = std::chrono::high_resolution_clock::now();
  double time_filter_ground =
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
          .count();
  ROS_INFO_STREAM("[PCD Converter] Time to remove ground from point cloud: "
                  << time_filter_ground << " s");

  // Remove outliers
  start_time = std::chrono::high_resolution_clock::now();

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.5);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pruned_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*pruned_cloud);

  end_time = std::chrono::high_resolution_clock::now();
  double time_outliers =
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
          .count();
  ROS_INFO_STREAM("[PCD Converter] Time to remove outliers from point cloud: "
                  << time_outliers << " s");

  // Put the point cloud in an octomap (information is still 3D up to now)
  octomap::OcTree tree_3d(p_resolution_);
  for (const auto& p : pruned_cloud->points)
  {
    tree_3d.updateNode(octomap::point3d(p.x, p.y, p.z), true);
  }
  tree_3d.updateInnerOccupancy();

  end_time = std::chrono::high_resolution_clock::now();
  double time_octomap = std::chrono::duration_cast<std::chrono::milliseconds>(
                            end_time - start_time)
                            .count();
  ROS_INFO_STREAM("[PCD Converter] Time to store point cloud in octomap: "
                  << time_octomap << " ms");

  // Store the tree to a binary file (to be read by octomap server)
  start_time = std::chrono::high_resolution_clock::now();

  tree_3d.writeBinary(p_output_file_);

  end_time = std::chrono::high_resolution_clock::now();
  double time_store = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end_time - start_time)
                          .count();
  ROS_INFO_STREAM(
      "[PCD Converter] Time to store octomap to file: " << time_store << " ms");

  // Output the pointcloud as message
  sensor_msgs::PointCloud2 pcl_msg_pruned;
  pcl::toROSMsg(*pruned_cloud, pcl_msg_pruned);
  pcl_msg_pruned.header.frame_id = p_frame_id_;

  ROS_WARN("[PCD Converter] Publishing the point clouds for 10 s");
  const double max_time = 10; // [s]
  double time = 0;            // [s]
  while (ros::ok() && time < max_time)
  {
    //
    pruned_pcl_pub_.publish(pcl_msg_pruned);
    original_pcl_pub_.publish(pcl_msg_old);
    //
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    time += 0.5;
  }

  ROS_WARN("[PCD Converter] Shutting down!");
  ROS_WARN("[PCD Converter] Press Ctrl+C!\n");
}

bool PcdConverter::readParameters()
{
  if (!nh_private_.getParam("input_file", p_input_file_))
  {
    ROS_WARN("[PCD Converter] Input file not specified");
    return false;
  }
  if (!nh_private_.getParam("output_file", p_output_file_))
  {
    ROS_WARN("[PCD Converter] Output file not specified");
    return false;
  }
  if (!nh_private_.getParam("resolution", p_resolution_))
  {
    ROS_WARN("[PCD Converter] Resolution not specified");
    return false;
  }
  if (!nh_private_.getParam("frame_id", p_frame_id_))
  {
    ROS_WARN("[PCD Converter] Frame id name not specified");
    return false;
  }
  if (!nh_private_.getParam("rot_yaw", p_rot_yaw_))
  {
    ROS_WARN("[PCD Converter] Rotation yaw not specified - using 0 deg");
    p_rot_yaw_ = 0;
  }
  if (!nh_private_.getParam("rot_roll", p_rot_roll_))
  {
    ROS_WARN("[PCD Converter] Rotation roll not specified - using 0 deg");
    p_rot_roll_ = 0;
  }
  if (!nh_private_.getParam("rot_pitch", p_rot_pitch_))
  {
    ROS_WARN("[PCD Converter] Rotation pitch not specified - using 0 deg");
    p_rot_pitch_ = 0;
  }
  return true;
}

} // end namespace smb_navigation
