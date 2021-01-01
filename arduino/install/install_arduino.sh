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


function install_arduino_dependencies {
	echo_green " Installing ARDUINO dependencies"

	echo_green " Installing ARDUINO IDE"
    mkdir ~/Arduino
    cd ~/Arduino
    # https://tttapa.github.io/Pages/Ubuntu/Software-Installation/Arduino/Arduino-IDE.html
    wget https://downloads.arduino.cc/arduino-1.8.10-linux64.tar.xz
    tar -xf arduino-1.8.10-linux64.tar.xz
    sudo mv arduino-1.8.10/ ~/arduino
     cd  ~/arduino/
    sudo ./install.sh
    rm ../arduino-1.8.10.tar.xz

    echo_green " Installing ARDUINO ROS dependencies"
    sudo apt-get install ros-melodic-rosserial-arduino -y
    sudo apt-get install ros-melodic-rosserial -y
	echo_green " Installing ARDUINO dependencies...DONE"
}

function install_dinamyxel { 
  echo_green " Installing  library"
  cd $HOME/src
  mkdir dynamixel
  cd dynamixel

  git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
  git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
  git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git

  cd $HOME/ros/catkin_ws/src

  ln -s $HOME/src/dynamixel .
  ln -s /home/ubuntu/src/MARRtinoSocial/face_tracker_pkg/ .
  ln -s /home/ubuntu/src/MARRtinoSocial/face_tracker_control/ .
  ln -s /home/ubuntu/src/MARRtinoSocial/dynamixel_motor/ .
  
  cd ..
  echo_green " Compilazione"
  catkin_make
 echo_green " Installazione ultimata"
}


function install_opencm { 
  echo_green " Installing OPENCM "
  wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCM9.04/master/99-opencm-cdc.rules
  sudo cp ./99-opencm-cdc.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger

  #sudo apt-get install libncurses5-dev:i386

}


function main {

    #install_arduino_dependencies
    install_dinamyxel
}


main
