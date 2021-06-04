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
    sudo apt-get install htop -y
    sudo apt install catkin -y

   

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
    sudo apt-get install ros-melodic-global-planner -y
    sudo apt-get install ros-melodic-yocs-velocity-smoother
    sudo apt-get install ros-melodic-apriltag-ros -y
    sudo apt-get install ros-melodic-usb-cam -y
    sudo apt-get install ros-melodic-teleop-twist-keyboard -y
    sudo apt-get install ros-melodic-web-video-server -y
    sudo apt-get install ros-melodic-rosbridge-server  -y 
    sudo apt-get install ros-melodic-rtabmap-ros -y
    sudo apt-get install ros-melodic-joint-state-publisher-gui -y
    sudo apt-get install ros-melodic-cartographer-ros -y
    sudo apt-get install ros-melodic-serial -y
    #sudo apt-get install ros-melodic-teraranger -y
    
   

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

function install_webserver {
sudo apt install tmux nginx -y
cd /var/www/html
sudo cp $HOME/src/marrtino_r3d/install/index.html .
sudo ln -s $HOME/src/marrtino_r3d/www/teleop teleop


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
echo_green " Install JOY"
cd $HOME/src
git clone https://github.com/Imperoli/gradient_based_navigation.git
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/gradient_based_navigation/ .
 

cd $HOME/ros/catkin_ws
echo_green " Compiling"
catkin_make

echo_green " Installing ..DONE"
}

function install_teraranger {


# install Teraranger
# https://github.com/Terabee/teraranger
echo_green " Install teraranger"
sudo apt-get install ros-melodic-teraranger-array -y
cd $HOME/src
git clone https://github.com/Terabee/teraranger.git
cd $HOME/ros/catkin_ws/src

ln -s $HOME/src/teraranger/ .
 

cd $HOME/ros/catkin_ws
echo_green " Compiling"
catkin_make
# da i permessi alla porta
sudo chmod a+rw /dev/ttyACM0
echo_green " Installing ..DONE"
}




function install_marrtinoR3D {



#echo “export ROS_PACKAGE_PATH=/home/ubuntu/ros/catkin_ws/src:/opt/ros/melodic/share” >> ~/.bashrc
#echo "source /home/ubuntu/ros/catkin_ws/devel/setup.bash" >> ~/.bashrc
#echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
#echo "export MARRTINO_VERSION=4.0" >> ~/.bashrc
#echo "export MARRTINO_APPS_HOME=/home/ubuntu/src/marrtino_apps" >> ~/.bashrc

echo_green " Installing ..DONE"
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

function install_lidar {

echo_green "********* LIDAR INSTALLATION **********"
echo_green "***************************************"
cd $HOME/src
git clone  https://github.com/Slamtec/rplidar_ros.git
cd $HOME/ros/catkin_ws/src
ln -s /home/ubuntu/src/rplidar_ros/ .
cd $HOME/ros/catkin_ws
catkin_make -j1
cd ..
echo_green "************* END INSTALLATION **************"
echo_green "*********************************************"

}

function install_marrtinoapp {

echo_green " Install MARRtinoapp"
cd $HOME/src
git clone https://bitbucket.org/iocchi/marrtino_apps.git
#git clone https://bitbucket.org/iocchi/stage_environments.git


#cd $HOME/ros/catkin_ws/src
#ln -s $HOME/src/stage_environments/ .
#cd $HOME/ros/catkin_ws
#catkin_make -j1


}

function install_spqrel_navigation {
cd $HOME/src
git clone https://github.com/LCAS/spqrel_navigation.git
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/spqrel_navigation .
cd ..
catkin_make
}

function install_newboard {
echo_green " Install Newboard"
mkdir $HOME/src/srrg
cd $HOME/src/srrg/
git clone https://gitlab.com/srrg-software/srrg2_orazio.git
cd srrg2_orazio

# compile firmware
cd  ~/src/srrg/srrg2_orazio/srrg2_orazio/firmware_build/atmega2560
make 

cd $HOME/ros/catkin_ws
catkin_make -j1

}

function install_hectorslam {
echo_green "********* HECTOR SLAM INSTALLATION **********"
echo_green "*********************************************"
cd $HOME/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
sudo -H apt-get install -y libqt4-dev
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/hector_slam .
cd ..
catkin_make
cd ..
echo_green "************* END INSTALLATION **************"
echo_green "*********************************************"
}

function install_navigationtutorial {

echo_green " Install Navigation Tutorial"
cd $HOME/src
git clone https://github.com/ros-planning/navigation_tutorials.git



cd $HOME/ros/catkin_ws/src
ln -s /home/ubuntu/src/navigation_tutorials/navigation_stage/ .
cd $HOME/ros/catkin_ws
catkin_make -j1


}
function install_talk {

echo_green " Install Navigation Tutorial"
cd $HOME/src
git clone https://github.com/artigianitecnologici/talk



cd $HOME/ros/catkin_ws/src
ln -s /home/ubuntu/src/talk / .
cd $HOME/ros/catkin_ws
catkin_make -j1


}


function main {


     
    install_basics
    install_dependencies
    install_think_driver
    install_joy
    install_webserver
    install_marrtinoR3D
    install_arduino_dependencies
    install_lidar
    install_newboard
    install_marrtinoapp
    install_hectorslam
    install_navigationtutorial
    install_webserver
    install_teraranger
    install_spqrel_navigation
    install_talk
}


main
