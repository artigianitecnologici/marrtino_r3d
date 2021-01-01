#!/usr/bin/env bash
echo "********* REALSENSE T265 INSTALLATION **********"
echo "*********************************************"
sudo apt update
sudo apt-get install ros-melodic-realsense2-camera
sudo apt-get install ros-melodic-realsense2-description
echo "**** install rules ****"

sh setup_udev_rules.sh
echo "************* END INSTALLATION **************"
echo "*********************************************"

