#!/bin/bash

# User input
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color


# Parameters - TODO
resolution=0.2 # [m]
z_min=-0.5
z_max=1.0

# Check inputs
print_help () {
  echo -e "\n${YELLOW}Function usage:${NC} ./pcd_to_gridmap.sh abs_path_input_file abs_path_output_folder [run_rviz]\n"
}

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  print_help
  exit 1
elif [ "$#" -lt 2 ]; then
  echo -e "${RED}FAILURE! Please specify the path to the data and the name of the input file${NC}"
  print_help
  exit -1
fi

input_file="$1"  # This has to be a *.pcd file!
data_path="$2"

if [ "$#" -eq 3 ]; then
  run_rviz=$3
else
  run_rviz=false
fi

output_bt_file="${data_path}/temp_binary_tree.bt"

# Generate a binary tree using octomap
echo -e "${YELLOW}Generating binary tree using OctoMap -> Press Ctrl+C when converter is done${NC}"
sleep 3s

if [ $run_rviz = true ] ; then
  rviz -d pcd_converter.rviz &
  sleep 2s
fi

roslaunch smb_navigation pcd_converter.launch resolution:=${resolution} input_file:=${input_file} output_file:=${output_bt_file}
sleep 1s
echo -e "${YELLOW}Binary tree generated!${NC}"

# Sleep and then run map saver
sleep 5s
echo -e "${YELLOW}Saving occupancy map to image${NC}"
rosrun map_server map_saver &

# Once it is done, sleep and then start octomap server
sleep 3s

echo -e "${YELLOW}Activating OctoMap Server${NC}"
roslaunch smb_navigation octomap_server.launch resolution:=${resolution} path:=${output_bt_file} z_min:=${z_min} z_max:=${z_max}

# Move the generated files to the output folder - the map is first generated and then moved (instead of being generated in the output folder directly) so that the relative path in the yaml file is correct by default
mv map.yaml ${data_path}/map.yaml
mv map.pgm ${data_path}/map.pgm

# Make sure that these nodes are shut down
sleep 3s
rosnode kill /pcd_converter_node /octomap_server /map_saver /rviz

echo -e "\n${YELLOW}Map has been generated in ${data_path}.${NC}"
echo -e "\n==================================================="
echo -e "${YELLOW}WARNING!${NC} Check that the output yaml file has no"
echo -e " 'nan' in the origin - otherwise replace with 0"
echo -e "==================================================="


