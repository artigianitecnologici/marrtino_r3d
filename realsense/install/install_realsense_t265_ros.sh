#!/usr/bin/env bash
echo "********* REALSENSE T265 INSTALLATION **********"
echo "*********************************************"
sudo apt update
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt-get install ros-$ROS_DISTRO-realsense2-description -y
echo "**** install rules ****"

sh setup_udev_rules.sh
echo "************* END INSTALLATION **************"
echo "*********************************************"

