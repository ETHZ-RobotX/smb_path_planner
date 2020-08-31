# Super Mega Bot Path Planner

Package for path planning of Super Mega Bots for the ETH Robotics Summer School. 
The package has been tested under ROS Melodic and Ubuntu 18.04.

__Author: Luca Bartolomei__  
__Affiliation: Vision For Robotics Lab, ETH Zurich__  
__Contact: Luca Bartolomei, lbartolomei@ethz.ch__

# TO-DO list
This is what should be done if something changes in the software stack of the summer school.
* ~~Adapt the planner with the new Gazebo simulation of the SMB~~ (03/06/2020)
* ~~Adapt the planner with the new controller of the SMB~~ (03/06/2020)
* Add possibility to estimate traversability of terrain (2.5D maps)
* Find proper way to increase the clearance of the global paths from costmap_2d
* Tune the planner properly
* Write configuration and launch files for the real robot (if necessary)
* ~~In order to run move_base stuff, we need odometry information. This should be added both in the simulation and in the state estimator of the real robot.~~ (03/06/2020)

# Installation instructions  
Install the following packages first:
```
sudo apt-get install ros-melodic-cmake-modules ros-melodic-velodyne-gazebo-plugins python-wstool python-catkin-tools ros-melodic-ompl ros-melodic-move-base ros-melodic-navfn ros-melodic-dwa-local-planner ros-melodic-costmap-2d ros-melodic-teb-local-planner ros-melodic-robot-self-filter ros-melodic-pointcloud-to-laserscan
```
Create a catkin workspace:  
```
mdkir -p ~/catkin_ws/src
cd ~catkin_ws
source /opt/ros/melodic/setup.bash
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```
Then, clone the repository in your catkin workspace:  
```
cd ~/catkin_ws/src
git clone git@github.com:VIS4ROB-lab/smb_path_planner.git
cd ~/catkin_ws
catkin build smb_path_planner
```  

## Additional documentation
Additional documentation on `move_base` can be found [here](https://wiki.ros.org/move_base), while for the local planner, refer to the TEB Local Planner instructions [here](https://wiki.ros.org/teb_local_planner).

## Structure of the code
The `smb_path_planner` package is composed by 3 packages:
* `smb_ompl_planner`: global planner for `move_base` based on the [OMPL library](http://ompl.kavrakilab.org/) for motion planning;
* `smb_navigation`: package containing utilities, configurations and launch files for the planning with `move_base`;
* `smb_navigation_rviz`: package containing the RViz plugin to put a goal for the planner easily.

## Planning Panel in RViz
Make sure all the packages have built successfully. As a sanity check, 
re-source your workspace (`$ source ~/catkin_ws/devel/setup.bash`) and start 
up RViz (`$ rviz`).
In RViz, select `Panels -> Add New Panel` and select `Planning Panel` under 
`smb_navigation_rviz`.

Next, under `Displays`, add an `InteractiveMarkers` display with the topic 
`/planning_markers/update`. You should be able to see the interactive markers
 and the planning panel.

## How to run the planner in simulation
First, start the simulation in Gazebo, RViz and RQT Plugin to select the 
controller, by running:
```
$ roslaunch smb_sim sim_path_planner.launch
```
In the controller panel, select `SmbPathFollowingController` from 
the list. If this controller does not show up, press the refresh button and 
try again. To start the controller, press the play button.  
Finally, start the local and global planners:
```
$ roslaunch smb_navigation navigate2d_ompl.launch
```
To send a global goal position, select it in the planning panel and press the
button `Start Planning`.

## How to run the planner on the real robot
Start the `LPC` on the robot and the `OPC` on the operator side. To run the 
`LPC`, run the following commands in the PC of the robot in multiple terminals:
```
$ roscore # terminal 1
$ roslaunch smb_lpc lpc.launch # terminal 2
```
Start the LiDAR and the mapping in another terminal in the robot PC:
```
$ roslaunch ethzasl_icp_mapper supermegabot_robosense_dynamic_mapper.launch # terminal 3
``` 
 Then to start the planners, run on the PC of the robot in a separate terminal:
```
$ roslaunch smb_navigation navigate2d_ompl.launch # terminal 5
```
On the operator side, start the `OPC`. First connect the `rosmaster` of the 
PC to the `rosmaster` of the robot and then run in a terminal:
```
$ roslaunch smb_opc opc.launch
```
To send a global goal position, follow the same procedure as in simulation:
1. Select the `SmbPathFollowingController` in the control panel and start it;
2. Send the global goal using the `Planning Panel` in RViz.
