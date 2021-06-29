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
 * pcd_converter.h
 * @brief Header of the class that reads a PCD and outputs a point cloud pruned
 *        of the ground and of outliers points. This point cloud is store in an
 *        octomap
 * @author: Luca Bartolomei
 * Created on: June 08, 2021
 *
 */

#include <ros/ros.h>

namespace smb_navigation
{

class PcdConverter
{

public:
  PcdConverter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  ~PcdConverter() {}

  bool isInitialized() const { return initialized_; }

  void generateMap();

private:
  // Auxiliaries

  /**
   * @brief Function to read parameters from server
   * @return True if all parameters have been found, False otherwise
   */
  bool readParameters();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pruned_pcl_pub_;
  ros::Publisher original_pcl_pub_;

  // Parameters and auxiliaries
  std::string p_input_file_;
  std::string p_output_file_;

  double p_resolution_;
  std::string p_frame_id_;

  float p_rot_yaw_;
  float p_rot_roll_;
  float p_rot_pitch_;

  bool initialized_;
}; // end class PcdConverter

} // end namespace smb_navigation
