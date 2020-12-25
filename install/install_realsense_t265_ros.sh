#!/usr/bin/env bash
echo "********* HECTOR SLAM INSTALLATION **********"
echo "*********************************************"
cd $HOME/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/realsense-ros .
cd ..
catkin_make
cd ..
echo "************* END INSTALLATION **************"
echo "*********************************************"

