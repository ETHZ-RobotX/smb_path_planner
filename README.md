# Super Mega Bot Path Planner

Package for path planning of Super Mega Bots for the first ETH Robotics 
Summer School, 2019. 
The package has been tested under ROS Melodic and Ubuntu 18.04.

__Author: Luca Bartolomei__  
__Affiliation: Vision For Robotics Lab, ETH Zurich__  
__Contact: Luca Bartolomei, lbartolomei@ethz.ch__
# Getting Started
If the packages from the repository 
[`ethz-asl/eth_robotics_summer_school_2019`](https://github.com/ethz-asl/eth_robotics_summer_school_2019) have already been downloaded and 
built, it is possible to skip the "Installation" section and go directly to the
["Structure of the code"](#structure-of-the-code).  
If this is not the case, follow the installation instructions below.
## Installation
This package is intended to be used with Ubuntu 18.04 and [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or above.
After installing ROS, install some extra dependencies:
```asm
$ sudo apt-get install ros-melodic-cmake-modules ros-melodic-ompl ros-melodic-costmap-2d ros-melodic-grid-map ros-melodic-grid-map-visualization ros-melodic-velodyne-gazebo-plugins python-wstool python-catkin-tools libyaml-cpp-dev protobuf-compiler autoconf
```
Then if not already done so, set up a new catkin workspace:
```asm
$ source /opt/ros/melodic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin config --merge-devel
```
In this installation guide, it is assumed that the following packages for 
simulation have already been downloaded from the SMB monorepo and built in the 
same catkin workspace:
* `any_node`
* `message_logger`
* `kindr`
* `kindr_ros`
* `smb_sim`
* `smb_path_following_controller`
* `elevation_mapping`
* `elevation_mapping_demo`
* `traversability_estimation`

To download all the other necessary dependencies for the path planner, run 
the following script in the source folder of your catkin workspace, by 
specifying the destination `src_workspace_path` of the clone procedure:
```asm
$ chmod +x smb_path_planner/bin/clone_repos.sh
$ ./smb_path_planner/bin/clone_repos.sh -f src_workspace_path
```
To show the help of the installation script, make it executable and run:
```asm
$ ./smb_path_planner/bin/clone_repos.sh -h
```
Then, compile:
```asm
$ cd ~/catkin_ws
$ catkin build smb_path_planner
```
## Structure of the code
The `smb_path_planner` package is composed by 4 parts:
* `smb_planner_common`: this package implements all the utility functions for 
visualization, parameter reading and traversability estimation that are 
needed by both the global and the local planners.
* `smb_planner_rviz`: this package is the implementation for the planning 
panel in RViz.
* `smb_global_planner`: global planner package. It is possible to select 
different planning algorithms (RRT*, Informed RRT*, PRM) by modifying the 
parameter file `smb_planner_parameters.yaml`.
* `smb_local_planner`: local planner package. It implements the CHOMP solver 
with nonholonomic constraints.  

The parameters for the planners and the elevation and traversability mapping 
are stored in `smb_path_planner/smb_planner_common/cfg/`.

## Planning Panel in RViz
Make sure all the packages have built successfully. As a sanity check, 
re-source your workspace (`$ source ~/catkin_ws/devel/setup.bash`) and start 
up RViz (`$ rviz`).
In RViz, select `Panels -> Add New Panel` and select `Planning Panel` under 
`smb_planner_rviz`.

Next, under `Displays`, add an `InteractiveMarkers` display with the topic 
`/planning_markers/update`. You should be able to see the interactive markers
 and the planning panel.

## How to run the planner in simulation
First, start the simulation in Gazebo, RViz and RQT Plugin to select the 
controller, by running:
```asm
$ roslaunch smb_sim sim_path_planner.launch
```
In the controller panel, select `SmbPathFollowingController` from 
the list. If this controller does not show up, press the refresh button and 
try again. To start the controller, press the play button.  
Then, start the `elevation_mapping` node:
```asm
$ roslaunch smb_local_planner smb_elevation_mapping_simulation.launch
``` 
Finally, start the local and global planners:
```asm
$ roslaunch smb_local_planner smb_planner_simulation.launch
```
To send a global goal position, select it in the planning panel and press the
button `Global Planner Service`. Once a global path has been computed and 
showed in RViz, start the local planner by pressing the button `Start Local 
Planner`.  

## How to run the planner on the real robot
Start the `LPC` on the robot and the `OPC` on the operator PC. All the necessary 
nodes should be started. Then in one terminal run:
```asm
$ roslaunch smb_local_planner smb_elevation_mapping_real.launch
```
This launch files starts the elevation mapping node. Then start the planners:
```asm
$ roslaunch smb_local_planner smb_planner_real.launch
```
To send a global goal position, follow the same procedure as in simulation:
1. Select the `SmbPathFollowingController` in the control panel and start it;
2. Send the global goal using the `Planning Panel` in RViz.
