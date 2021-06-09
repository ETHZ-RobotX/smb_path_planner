#!/bin/bash

# User input
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Generate a binary tree using octomap
echo -e "${YELLOW}Generating binary tree using OctoMap -> Press Ctrl+C when converter is done${NC}"
sleep 5s

roslaunch smb_navigation pcd_converter.launch
sleep 1s
echo -e "${YELLOW}Binary tree generated!${NC}"

# Once it is done, sleep and then start octomap server
sleep 3s

echo -e "${YELLOW}Activating OctoMap Server${NC}"
roslaunch smb_navigation octomap_server.launch &

# Sleep and then run map saver
sleep 3s
echo -e "${YELLOW}Saving occupancy map to image${NC}"
rosrun map_server map_saver

# Killall
sleep 3s
rosnode kill -a

echo -e "${YELLOW}Map has been generated in smb_path_planner/smb_navigation/script.${NC}"

