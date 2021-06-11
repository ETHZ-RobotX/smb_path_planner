# Super Mega Bot Path Planner

Package for path planning of Super Mega Bots for the ETH Robotics Summer School. 
The package has been tested under ROS Noetic and Ubuntu 20.04.

__Author__: Luca Bartolomei  
__Affiliation__: Vision For Robotics Lab, ETH Zurich  
__Contact__: Luca Bartolomei, lbartolomei@ethz.ch  

# Installation instructions  
Install the following packages first:
```
$ sudo apt install ros-noetic-cmake-modules ros-noetic-velodyne-gazebo-plugins ros-noetic-ompl ros-noetic-move-base ros-noetic-navfn ros-noetic-dwa-local-planner ros-noetic-costmap-2d ros-noetic-teb-local-planner ros-noetic-robot-self-filter ros-noetic-pointcloud-to-laserscan ros-noetic-ros-numpy ros-noetic-octomap-ros ros-noetic-octomap-server ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-grid-map-costmap-2d ros-noetic-grid-map-ros ros-noetic-map-server ros-noetic-global-planner
```
Then follow the instructions [here](https://github.com/ETHZ-RobotX/SMB_dev) to set up the simulation. Then, to build the planner:
```
$ catkin build smb_path_planner
```

## Additional documentation
Additional documentation on `move_base` can be found [here](https://wiki.ros.org/move_base), while for the local planner, refer to the TEB Local Planner instructions [here](https://wiki.ros.org/teb_local_planner). Finally, the documentation about `costmap_2d` is available [here](https://wiki.ros.org/costmap_2d).

## Structure of the code
The `smb_path_planner` package is composed by 3 packages:
* `smb_ompl_planner`: global planner for `move_base` based on the [OMPL library](http://ompl.kavrakilab.org/) for motion planning;
* `smb_navigation`: package containing utilities, configurations and launch files for the planning with `move_base`;
* `smb_navigation_rviz`: package containing the RViz plugin to put a goal for the planner easily;
* `traversability_layer`: custom `costmap_2d` implementation to incorporate traversability maps.

## Planning Panel in RViz
Make sure all the packages have built successfully. As a sanity check, re-source your workspace (`$ source ~/catkin_ws/devel/setup.bash`) and start up RViz (`$ rviz`). In RViz, select `Panels -> Add New Panel` and select `Planning Panel` under `smb_navigation_rviz`.

Next, under `Displays`, add an `InteractiveMarkers` display with the topic `/planning_markers/update`. You should be able to see the interactive markers and the planning panel.

## How to run the planner in simulation
First, start the simulation in Gazebo and RViz, by running:
```
$ roslaunch smb_gazebo sim.launch
```
Then, start the local and global planners. If you want to use RRTs as global planner, run:
```
$ roslaunch smb_navigation navigate2d_ompl.launch sim:=true
```
Otherwise, to use a standard global planner from `move_base`, run:
```
$ roslaunch smb_navigation navigate2d.launch sim:=true
```
To send a global goal position there a set of different possibilities:
* Set a goal with the planning panel and press the button `Start Planning`;
* Use RViz direcly by using the button `2D Nav Goal` and setting the goal pose;
* Publish directly on the topic `/move_base_simple/goal`.  

### How to run the planner in another frame
If the `world` frame is not available, it is possible to use the odometry frame for planning. for example, if the frame in use is called `odom`, run:
```
$ roslaunch smb_navigation navigate2d_ompl.launch global_frame:=odom
```

## How to run the planner on the real robot
Connect to the robot and start the state estimation and control pipeline. Once it is started, run the planner as before:
```
$ roslaunch smb_navigation navigate2d_ompl.launch
```
If necessary, set the right global and local frames used for planning.  

### Running with traversability estimation
**Note**: This component has not been fully tested yet!  
**Note**: In other to use this feature, make sure that the `traversability_estimation` package is installed ([link](https://github.com/leggedrobotics/traversability_estimation#installation)).

Start the simulation as in the previous case, and then run:
```
$ roslaunch smb_navigation navigate2d_ompl.launch run_traversability:=true
```
Notice that in this case, there are 3 different cost layers (static, laser scans and inflation layers), but **only** the static layer is active. 

It is also possible to use a custom layer (`traversability_layer`). To use it, follow the instructions in the configuration file `smb_navigation/config/move_base_costmaps/local_costmap_params_traversability.yaml`.  
In this case, notice that it is not possible to run the obstacle layer (based on laser scans) and the traversability layer at the same time in the current configuration, as the laser scan clears the traversability map. You are more than welcome to find a proper way to fuse these two maps!

## Troubleshooting
If there are problems due to linking against `pthread` or `boost`, build with following command:
```
$ catkin build --cmake-args -DBUILD_SHARED_LIBS=ON
```

