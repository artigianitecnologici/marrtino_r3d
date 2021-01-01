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

function install_librealsense {
	echo_green " Installing librealsense"
   cd $HOME/src
   git clone https://github.com/IntelRealSense/librealsense.git
   cd $HOME/src/librealsense
   mkdir  build  && cd build
   cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
    echo_green " Compilazione"
   make -j1
   sudo make install
 echo_green " Installazione ultimata"
}

function install_realsense {
	echo_green " Installing realsense"

sudo apt update
sudo apt-get install ros-melodic-realsense2-camera -y
sudo apt-get install ros-melodic-realsense2-description -y
 echo_green " Installazione realsense ultimata"



}

function install_realsense_rules {
echo_green  "Setting-up permissions for RealSense devices"

exec 3>&2
exec 2> /dev/null
con_dev=$(ls /dev/video* | wc -l)
exec 2>&3

if [ $con_dev -ne 0 ];
then
	echo -e "\e[32m"
	read -p "Remove all RealSense cameras attached. Hit any key when ready"
	echo -e "\e[0m"
fi

sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

echo_green  "udev-rules successfully installed"

echo_green " Installazione realsense rules ultimata"



}
function install_realsense_D435 {
echo_green  "install path  d435"

sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
cd $HOME/src/librealsense
./scripts/patch-realsense-ubuntu-lts.sh
#https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

echo_green " Installazione realsense path d435 ultimata"



}

function main {

    
    #install_librealsense
    #install_realsense
    #install_realsense_rules
    install_realsense_D435

}


main
