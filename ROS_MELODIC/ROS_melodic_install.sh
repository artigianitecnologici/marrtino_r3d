#!/usr/bin/env bash

RED='\e[41m'
GREEN='\e[42m'
RESET='\033[0m'

function echo_green {
    echo -e "\n${GREEN} $1 ${RESET}\n"
}

function echo_red {
    echo -e "\n${RED} $1 ${RESET}\n"
}

function install_ros {
	echo_green " Installing ROS Melodic"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-melodic-desktop-full -y
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
    printenv | grep ROS
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
	echo_green " Installing ROS Melodic...DONE"
}

function install_basics {
    echo_green " Installing basics"
    sudo apt install python-pip -y
    sudo apt install terminator -y
    sudo apt install git -y
    sudo pat install simplescreenrecorder -y
    echo_green " Installing basics...DONE"
}

function main {

    set -e
    install_basics
    install_ros
    set +e
}


main
