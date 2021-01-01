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
function install_mavros {
   echo_green " Installing MAVROS"
   sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
   echo_green " Installing lib"
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   sudo bash ./install_geographiclib_datasets.sh

   echo_green "MAVPROXY INSTALLATION"
   sudo apt-get install python-dev python-opencv python-wxgtk4.0 python-pip python-matplotlib python-lxml python-pygame
   pip install PyYAML mavproxy --user 
   echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
   sudo usermod -a -G dialout ubuntu

}

function main {

    install_mavros
    #install_dinamyxel
}


main
