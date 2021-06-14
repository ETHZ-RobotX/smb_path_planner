#!/usr/bin/env python3
""" 
@file   goal_from_plane.py
@brief  ROS Node that generates a goal for move base from a path on a plane
@author Luca Bartolomei, V4RL
@date   04.11.2020
"""

import numpy as np
from numpy.linalg import svd
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


do_plots = False
frame_id = "world"
distance_from_wall = 5.0
normal_direction = 1  # This decides the normal direction we consider


def planeFit(points):
    """
    p, n = planeFit(points)

    Given an array, points, of shape (d,...)
    representing points in d-dimensional space,
    fit an d-dimensional plane to the points.
    Return a point, p, on the plane (the point-cloud centroid),
    and the normal, n.
    """    
    # Collapse trialing dimensions
    points = np.reshape(points, (np.shape(points)[0], -1)) 
    assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0])
    ctr = points.mean(axis=1)
    x = points - ctr[:,np.newaxis]
    M = np.dot(x, x.T) # Could also use np.cov(x) here.
    return ctr, svd(M)[0][:,-1]
    
def path_callback(path_msg):
    # Check path frame
    path_frame = path_msg.header.frame_id
    if frame_id != path_frame:
        rospy.logerr("Path frame ID is different from specified frame")
        rospy.logerr("No goal will be generated")
        return

    rospy.loginfo("Received path on the plane")
    points = np.array([])
    for pose in path_msg.poses:
        position = np.array([pose.pose.position.x, 
                             pose.pose.position.y, 
                             pose.pose.position.z])
        if points.shape[0] == 0:
            points = position
        else:
            points = np.vstack((points, position))
    
    # Debug
    # points = np.array(
    #   [[0.0, 1.0, 1.0], [0.0, 2.0, 2.0], [0.0, 3.0, 3.0], [0.0, 4.0, 4.0],
    #    [0.0, 5.0, 1.0], [0.0, 6.0, 2.0], [0.0, 7.0, 3.0], [0.0, 8.0, 4.0]])
    
    rospy.loginfo("This path has {} points".format(points.shape[0]))
    centroid, normal = planeFit(points.T)
    
    # Compute the goal position: the goal is at a fixed distance from the 
    # centroid along the normal, with orientation such that we face the wall
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = frame_id
    goal_msg.header.stamp = rospy.Time.now()
    
    goal_msg.pose.position.x = normal_direction * distance_from_wall * normal[0] + centroid[0]
    goal_msg.pose.position.y = normal_direction * distance_from_wall * normal[1] + centroid[1]
    goal_msg.pose.position.z = normal_direction * distance_from_wall * normal[2] + centroid[2]
    
    # Since the path is in CAD world frame, we can project the normal on the 
    # (x,y) plane and compute the yaw from that.
    normal_xy = -normal_direction * normal[0:2] / np.linalg.norm(normal[0:2])
    yaw = math.atan2(normal_xy[1], normal_xy[0])
    rot = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False)
    rot_quat = rot.as_quat()
    goal_msg.pose.orientation.x = rot_quat[0]
    goal_msg.pose.orientation.y = rot_quat[1]
    goal_msg.pose.orientation.z = rot_quat[2]
    goal_msg.pose.orientation.w = rot_quat[3]
    
    goal_pub.publish(goal_msg)
    
    # Inform user
    rospy.loginfo("Plane Centroid: {}".format(centroid))
    rospy.loginfo("Plane Normal  : {}".format(normal))
    rospy.loginfo("Yaw           : {}".format(yaw))
    rospy.loginfo("Quaternion    : {}".format(rot_quat))
    
    # Plotting
    if do_plots:
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points[:, 0], points[:, 1], points[:, 2])
        ax.scatter(goal_msg.pose.position.x, goal_msg.pose.position.y,
                   goal_msg.pose.position.z)
        ax.quiver(goal_msg.pose.position.x, goal_msg.pose.position.y,
                  goal_msg.pose.position.z, yaw, 0.0, 0.0, length=1.0, normalize=True)
        ax.set_xlabel('X'), ax.set_ylabel('Y'), ax.set_zlabel('Z')
        plt.show()


if __name__ == '__main__':
    try:
        rospy.init_node('goal_from_plane', anonymous=True)
        
        # Parameters
        if rospy.has_param('~frame_id'):
            frame_id = rospy.get_param("~frame_id")
            
        if rospy.has_param('~distance_from_wall'):
            distance_from_wall = rospy.get_param("~distance_from_wall")
            
        if rospy.has_param('~normal_direction'):
            normal_direction = rospy.get_param("~normal_direction")
            assert(normal_direction == 1 or normal_direction == -1)
            
        if rospy.has_param('~do_plots'):
            do_plots = rospy.get_param("~do_plots")
        
        # Subscribers and Publishers
        path_sub = rospy.Subscriber("input_path", Path, path_callback)
        goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10,)
        rospy.loginfo("Waiting for path message...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
