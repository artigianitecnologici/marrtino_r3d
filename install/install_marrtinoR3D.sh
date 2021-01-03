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


function install_basics {
    echo_green " Installing basics"
    sudo apt install python-pip -y
    sudo apt install terminator -y
    sudo apt install git -y
    sudo apt-get install htop
  
    echo_green " Installing basics...DONE"
}

function install_dependencies {
	echo_green " Installing  dependencies"
    # Python modules

    # ROS packages
    sudo apt-get install ros-melodic-map-server -y
    sudo apt-get install ros-melodic-amcl -y
    sudo apt-get install ros-melodic-gmapping -y
    sudo apt-get install ros-melodic-joy -y
    sudo apt-get install ros-melodic-pid -y
    sudo apt-get install ros-melodic-move-base -y
    sudo apt-get install ros-melodic-dwa-local-planner -y
    sudo apt get install ros-melodic-global-planner -y
    sudo apt-get install ros-melodic-yocs-velocity-smoother
    sudo apt-get install ros-melodic-apriltag-ros -y
    sudo apt-get install ros-melodic-usb-cam -y
    sudo apt-get install ros-melodic-teleop-twist-keyboard -y
    sudo apt-get install ros-melodic-web-video-server -y
    sudo apt-get install ros-melodic-rosbridge-server  -y
    # 
    # install camera calibration
    cd $HOME/src
    git clone https://github.com/ros-perception/image_pipeline
    #
    rosdep install rosbridge_server
    sudo apt-get install ros-melodic-rosbridge-suite -y
    source /opt/ros/melodic/setup.bash

	echo_green " Installing dependencies...DONE"
}



function install_think_driver {


# install think driver

cd $HOME/src
git clone https://bitbucket.org/ggrisetti/thin_drivers.git
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/thin_drivers/thin_state_publisher/ .
ln -s $HOME/src/thin_drivers/thin_msgs/ .

cd $HOME/ros/catkin_ws
echo_green " Compiling"
catkin_make

echo_green " Installing ..DONE"
}

function install_joy {


# install joy
#sudo apt-get install ros-melodic-joy -y

cd $HOME/src
git clone https://github.com/Imperoli/gradient_based_navigation.git
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/gradient_based_navigation/ .
 

cd $HOME/ros/catkin_ws
echo_green " Compiling"
catkin_make

echo_green " Installing ..DONE"
}




function install_marrtinoR3D {

cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/marrtino_r3d/ .
cd $HOME/ros/catkin_ws
echo_green " Compiling ......"
catkin_make

#echo “export ROS_PACKAGE_PATH=/home/ubuntu/ros/catkin_ws/src:/opt/ros/melodic/share” >> ~/.bashrc
echo "source /home/ubuntu/ros/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "export MARRTINO_VERSION=4.0" >> ~/.bashrc
echo "export MARRTINO_APPS_HOME=/home/ubuntu/src/marrtino_apps" >> ~/.bashrc

echo_green " Installing ..DONE"
}


function main {


     
    #install_basics
    #install_dependencies
    #install_think_driver
    #install_joy
    #install_webserver
    install_marrtinoR3D
}


main
