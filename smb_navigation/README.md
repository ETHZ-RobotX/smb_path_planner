### smb_navigation  
ROS package that runs the `move_base` stack for the SMB. More information about the `move_base` package can be found [here](https://wiki.ros.org/move_base).

Install these deps from apt:  

`sudo apt install ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins python-wstool python-catkin-tools ros-noetic-ompl ros-noetic-move-base ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-costmap-2d ros-noetic-teb-local-planner ros-noetic-robot-self-filter ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy ros-noetic-grid-map-costmap-2d`



Before running make sure you build the package

`catkin build smb_navigation_scripts`

### Path Planning from CMU stack
```bash
# use o3d, default rviz
roslaunch smb_gazebo sim.launch
roslaunch smb_slam online_slam.launch use_sim_time:=true
roslaunch smb_navigation navigate2d_cmu.launch smb:=false

# use graph_msf, change the global fram in rviz to world_graph_msf
roslaunch smb_gazebo sim.launch
roslaunch smb_msf_graph smb_msf_graph.launch use_sim_time:=true
roslaunch smb_navigation navigate2d_cmu.launch use_msf:=true global_frame:=world_graph_msf state_estimation_topic:=/transformed_odom smb:=false
```

