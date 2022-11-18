#!/usr/bin/env bash
echo "**** ROSBAG PLAY ****"
echo "*********************"
cd $HOME/playground
rosparam set use_sim_time true
rosbag play $.bag -d 5 --clock

