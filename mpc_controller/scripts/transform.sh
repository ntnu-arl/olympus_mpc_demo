#!/bin/bash

if [ $# -eq 0 ]; then
  echo "No rosbag name provided. Using default."
  rosbag_name="olympus_mpc_rosbag"
elif [ $# -eq 1 ]; then 
    rosbag_name=$1 
fi
cd $rosbag_name

#record `/joint_states` to playback motions if needed
#record all legs for debugging if needed



rostopic echo -b $rosbag_name.bag /leg2_node/motor_statuses   -p > $rosbag_name\_motor_statuses.txt
rostopic echo -b $rosbag_name.bag /leg2_node/command_position -p > $rosbag_name\_reference.txt

rostopic echo -b $rosbag_name.bag /qualisys/olympus/odom      -p > $rosbag_name\_odom.txt
rostopic echo -b $rosbag_name.bag /olympus/desired_angle      -p > $rosbag_name\_target.txt

rostopic echo -b $rosbag_name.bag /leg_mpc_status             -p > $rosbag_name\_mpc_status.txt
rostopic echo -b $rosbag_name.bag /controller_current_phase   -p > $rosbag_name\_controller_phase.txt

echo "Exported the topics in txts"
