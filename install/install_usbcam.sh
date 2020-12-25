#!/bin/bash
#
# ROS usb_cam driver
echo "Install ROS usb_cam driver "
#
sudo apt update
yes | sudo apt install ros-melodic-usb-cam
#
#
echo "Install HTOP"
yes | sudo apt install htop
#

