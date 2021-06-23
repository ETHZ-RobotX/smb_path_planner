import rospkg
import subprocess
import os

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

print('YOU NEED TO HAVE A ROSCORE RUNNING')

# get the file path for rospy_tutorials
mapPath = rospack.get_path('smb_slam') + '/compslam_map.pcd'
smb_nav_path = rospack.get_path('smb_navigation')
output_path = smb_nav_path + '/data/test'
command_string = smb_nav_path + '/script/pcd_to_gridmap.sh ' + mapPath + ' ' + output_path
os.system(command_string) 