#!/bin/bash

echo "Running simulation..."
SCRIPT_DIR=$(dirname $(readlink -f "$0"))
cd $SCRIPT_DIR
while [ $(pwd | grep -c 'src') -gt 0 ]; do
  # Move up one directory
  cd ../
done

ws_root=$(realpath .)
echo "ROS workspace: $ws_root"
cd $ws_root

source devel/setup.bash

roscd olympus_drake_ros
cd ../olympus_drake
rosrun olympus_drake_ros olympus_drake_ros