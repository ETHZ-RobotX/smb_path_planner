#!/usr/bin/env bash

# Initialize flags and inputs.
didDefineGitFolder=0
git_folder=""

# A method which displays information on how to use this script.
print_help() {
 echo " "
 echo "This script will clone all the necessary repositories to run SMB path "
 echo "planner in a simulated environment. Make sure you have followed all the"
 echo "other instructions before running this script!"
 echo ""
 echo "The script can be used in the following way: "
 echo "                 ./clone_repos.sh -f destination_folder_path."
 echo "If no destiantion folder is specified, the script will return an error." 
 echo " "
 echo "Here we assume the following repos are already in the workspace:"
 echo " - smb_common"
 echo " - any_node"
 echo " - message_logger"
 echo " - kindr"
 echo " - kindr_ros"
 echo " - grid_map"
 echo " "
 echo "The other necessary repositories will be cloned only if the are not "
 echo "already present in the destination folder. All the repositories are ".
 echo "cloned with the htpps protocol."
 echo " "
}

# Check we have an input argument
if [ "$1" == "" ]
then
	echo "Missing argument, use script as ./clone_repos.sh -f destination_folder_path"
	exit 1
fi 

# Check for options.
while getopts "f:h" opt; do
  case $opt in
	f)
	  print_help
	  didDefineGitFolder=1
	  git_folder=$OPTARG >&2
	echo "User specifiec folder $git_folder"
	  ;;
	h)
	 print_help
	 exit 1
	  ;;
	\?)
	  echo "Invalid option: -$OPTARG" >&2
	  echo "=============================================================================="
	  echo "HELP INFORMATION:"
	  print_help
	 exit 1
	  ;;
	:)
	  echo "Option -$OPTARG requires as argument the folder name where to clone the repos." >&1
	  echo "=============================================================================="
	  echo "HELP INFORMATION:"
	  print_help
	  exit 1
	  ;;
  esac
done

# If no option was provided for the git folder, use a default one.
if [ $didDefineGitFolder -ne 1 ]; then
  echo "Destination folder was not provided. Aborting!"
  exit 1
fi

# Set the list of repositories to clone.
declare -a REPOSITORIES=(
  "eigen_catkin"
  "eigen_checks"
  "glog_catkin"
  "ceres_catkin"
  "gflags_catkin"
  "catkin_simple"
  "minkindr"
  "minkindr_ros"
  "voxblox"
  "traversability_estimation"
  "elevation_mapping"
)

# Create a git folder if necessary and move to it.
mkdir -p $git_folder
cd $git_folder

echo "Starting to clone repos here: $git_folder"
echo ""

for REPOSITORY in "${REPOSITORIES[@]}"; do
  echo -e "Checking repository \033[1;34m${REPOSITORY}\033[0m"
  if [ -d "${REPOSITORY}" ]; then
	  echo -e "  The folder exists. Skiping repository."
  else
	  echo -e "  The folder doesn't exist. Cloning repository."
	
		if [ "${REPOSITORY}" == "eigen_catkin" ] || [ "${REPOSITORY}" == "eigen_checks" ]  || 
			 [ "${REPOSITORY}" == "glog_catkin" ] || [ "${REPOSITORY}" == "ceres_catkin" ]  || 
			 [ "${REPOSITORY}" == "gflags_catkin" ] || [ "${REPOSITORY}" == "minkindr" ]  || 
			 [ "${REPOSITORY}" == "minkindr_ros" ] || [ "${REPOSITORY}" == "voxblox" ]; then
			 git clone "https://github.com/ethz-asl/${REPOSITORY}.git"
		fi  
		
		if [ "${REPOSITORY}" == "catkin_simple" ]; then
			git clone "https://github.com/catkin/${REPOSITORY}.git"
		fi
		
		if [ "${REPOSITORY}" == "traversability_estimation" ]; then
			git clone "https://github.com/leggedrobotics/${REPOSITORY}.git"
		fi
		
		if [ "${REPOSITORY}" == "elevation_mapping" ]; then
			git clone "https://github.com/ANYbotics/${REPOSITORY}.git"
		fi
	fi
done
