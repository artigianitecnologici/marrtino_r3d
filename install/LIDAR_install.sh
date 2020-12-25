#!/usr/bin/env bash
echo "********* LIDAR INSTALLATION **********"
echo "***************************************"
cd $HOME/src
git clone  https://github.com/Slamtec/rplidar_ros.git
cd $HOME/ros/catkin_ws/src
ln -s /home/ubuntu/src/rplidar_ros/ .
cd $HOME/ros/catkin_ws
catkin_make -j1
cd ..
echo "************* END INSTALLATION **************"
echo "*********************************************"

