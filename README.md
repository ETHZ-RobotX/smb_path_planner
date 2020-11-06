# Super Mega Bot Path Planner

Package for path planning of Super Mega Bots for the ETH Robotics Summer School. 
The package has been tested under ROS Melodic and Ubuntu 18.04.

__Author__: Luca Bartolomei  
__Affiliation__: Vision For Robotics Lab, ETH Zurich  
__Contact__: Luca Bartolomei, lbartolomei@ethz.ch  

# Installation instructions  
Install the following packages first:
```
$ sudo apt-get install ros-melodic-cmake-modules ros-melodic-velodyne-gazebo-plugins python-wstool python-catkin-tools ros-melodic-ompl ros-melodic-move-base ros-melodic-navfn ros-melodic-dwa-local-planner ros-melodic-costmap-2d ros-melodic-teb-local-planner ros-melodic-robot-self-filter ros-melodic-pointcloud-to-laserscan ros-melodic-ros-numpy
$ pip install numpy matplotlib scipy
```
Then follow the instructions [here](https://github.com/ETHZ-RobotX/SMB_dev). **Note:** the project may not be publicly available yet. If not refer to the original ETHZ Summer School [repository](https://github.com/ethz-asl/eth_supermegabot).

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
First, start the simulation in Gazebo, RViz and RQT Plugin to select the 
controller, by running:
```
$ roslaunch smb_sim sim_path_planner.launch
```
In the controller panel, select `MpcTrackLocalPlan` from the list. If this controller does not show up, press the refresh button and try again. To start the controller, press the play button. Finally, start the local and global planners. If you want to use RRTs as global planner, run:
```
$ roslaunch smb_navigation navigate2d_ompl.launch
```  
Otherwise, to use a standard global planner from `move_base`, run:
```
$ roslaunch smb_navigation navigate2d.launch
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

### Running with traversability estimation
Start the simulation as in the previous case, and then run:
```
$ roslaunch smb_navigation navigate2d_ompl.launch run_traversability:=true
```
Notice that in this case, there are 3 different cost layers (static, laser scans and inflation layers), but **only** the static layer is active. 

It is also possible to use a custom layer (`traversability_layer`). To use it, follow the instructions in the configuration file `smb_navigation/config/move_base_costmaps/local_costmap_params_traversability.yaml`.  
In this case, notice that it is not possible to run the obstacle layer (based on laser scans) and the traversability layer at the same time in the current configuration, as the laser scan clears the traversability map. You are more than welcome to find a proper way to fuse these two maps!

## How to run the planner on the real robot
Connect to the robot and start the state estimation and control pipeline. Once it is started, run the planner as before:
```
$ roslaunch smb_navigation navigate2d_ompl.launch
```  
If necessary, set the right global frame used for planning.

## Troubleshooting
If there are problems due to linking against `pthread` or `boost`, build with following command:
```
$ catkin build --cmake-args -DBUILD_SHARED_LIBS=ON
```

