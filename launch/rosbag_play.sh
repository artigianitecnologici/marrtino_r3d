#!/usr/bin/env bash
echo "*********************"
echo "**** ROSBAG PLAY ****"
echo "*********************"
cd $HOME/src/marrtino_r3d/maps
rosparam set use_sim_time true
rosbag play casaodom.bag --clock

