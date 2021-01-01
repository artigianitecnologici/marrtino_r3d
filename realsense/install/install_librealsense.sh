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



function main {

    
    install_librealsense

}


main
