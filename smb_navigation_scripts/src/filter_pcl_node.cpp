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
 * filter_pcl_node.cpp
 * @brief Node that filter the input point cloud to remove the ground and
 *        parts of the robot's body
 * @author: Luca Bartolomei
 * Created on: March 05, 2021
 */

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class FilterPcl
{
public:
  /**
   * @brief Constructor
   */
  FilterPcl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh), nh_private_(nh_private)
  {
    // Initialization
    pcl_sub_ =
        nh_.subscribe("rslidar_points", 10, &FilterPcl::pclCallback, this);
    pcl_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("rslidar_points_filtered", 1);

    // Read parameters
    nh_private_.param("ground_removal_th", k_ground_removal_th, 2.0);
    nh_private_.param("clear_radius", k_clear_radius, 2.0);
  }

  /**
   * @brief Destructor
   */
  ~FilterPcl() {}

private:
  // ROS Callback
  void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl_msg, *cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // remove ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud =
        removeOutlierLidarPoints(cloud);

    // Output message
    sensor_msgs::PointCloud2 pcl_out;
    pcl::toROSMsg(*filtered_cloud, pcl_out);

    pcl_out.header = pcl_msg->header;
    pcl_pub_.publish(pcl_out);
  }

  /**
   * @brief Method to remove the outliers points from the input pcl
   * @param[in] cloud : input point cloud to check
   * @return Point cloud without outliers
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  removeOutlierLidarPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < (*cloud).size(); i++)
    {
      // Check how far the point in the sensor frame is
      double dist_from_sensor =
          std::sqrt(cloud->points[i].x * cloud->points[i].x +
                    cloud->points[i].y * cloud->points[i].y +
                    cloud->points[i].z * cloud->points[i].z);
      if (dist_from_sensor > k_clear_radius &&
          cloud->points[i].z > k_ground_removal_th)
      {
        output->points.push_back(cloud->points[i]);
      }
    }
    return output;
  }

protected:
  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;

  double k_ground_removal_th;
  double k_clear_radius;
};

/*****************************************************************************/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_pcl_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  FilterPcl filter_pcl(nh, nh_private);

  ros::spin();
  return 0;
}
