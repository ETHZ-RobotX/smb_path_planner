#!/usr/bin/env python3
""" 
@file   filter_pointcloud.py
@brief  ROS Node that intercepts point cloud and removes the points in a sphere
        around the sensor
@author Luca Bartolomei, V4RL
@date   06.11.2020
"""

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import time
 
clear_radius = 2.0
ground_th = -0.5
    
def pcl_callback(pcl_in_msg):
    rospy.loginfo_once("Received first pointcloud")
    
    # Get the point in numpy format and filter them
    pcl_in_numpy = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl_in_msg)
    pcl_out_numpy = []
    for p in pcl_in_numpy:
        if np.linalg.norm(p) > clear_radius and p[2] > ground_th:
            pcl_out_numpy.append(p)
            
    # Transform into array
    pcl_out_numpy = np.array(pcl_out_numpy)  
    
    # Check size
    if pcl_out_numpy.shape[0] == 0:
        rospy.logwarn_throttle(5, "No points in filtered point cloud!")
        return
    
    # Generate a PointCloud2 message and publish it
    pcl_out_msg = pcl_in_msg
    pcl_out_msg.header.stamp = rospy.Time.now()
    
    pcl_out_msg.width = pcl_out_numpy.shape[0]
    pcl_out_msg.height = 1
    pcl_out_msg.point_step = 12
    pcl_out_msg.row_step = pcl_out_msg.point_step*pcl_out_numpy.shape[1]
    pcl_out_msg.data = pcl_out_numpy.astype(np.float32).tobytes()
    
    pcl_pub.publish(pcl_out_msg)
    

if __name__ == '__main__':
    try:
        rospy.init_node('filter_pointcloud', anonymous=True)
        
        # Parameters
        if rospy.has_param('~clear_radius'):
            clear_radius = rospy.get_param("~clear_radius")
        
        # Subscribers and Publishers
        pcl_sub = rospy.Subscriber("rslidar_points", PointCloud2, pcl_callback, 
                                  queue_size=1)
        pcl_pub = rospy.Publisher('rslidar_points_filtered', PointCloud2, 
                                  queue_size=1)
        rospy.loginfo("Waiting for first PCL message...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
