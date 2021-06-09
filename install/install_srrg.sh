# Docker file for RoboCup@HomeEducation and MARRtino apps
# ROS Melodic, navigation, perception & additional packages
# Version 4 - 9/8/2020




# MARRtino ROS node

mkdir -p $HOME/src/srrg && cd $HOME/src/srrg 
    git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git 
    git clone https://gitlab.com/srrg-software/srrg2_orazio.git


# srrg_mapper

mkdir -p $HOME/src/srrg && cd $HOME/src/srrg 

    git clone https://gitlab.com/srrg-software/srrg_core.git 
    cd srrg_core && git checkout a8f88898 && cd .. 
    git clone https://gitlab.com/srrg-software/srrg_scan_matcher.git 
    cd srrg_scan_matcher && git checkout 31e7c7ac && cd .. 
    git clone https://gitlab.com/srrg-software/srrg_mapper2d.git 
    cd srrg_mapper2d && git checkout 5ea162d1 && cd .. 
    git clone https://gitlab.com/srrg-software/srrg_mapper2d_ros.git 
    cd srrg_mapper2d_ros && git checkout 9aa14795 && cd .. 


# patches
cd $HOME/src/srrg/srrg_mapper2d_ros 
rm CMakeLists.txt package.xml 
wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/CMakeLists.txt 
wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/package.xml 
cd $HOME/src/srrg/srrg_mapper2d_ros/src 
    rm srrg_mapper2d_node.cpp message_handler.cpp 
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/srrg_mapper2d_node.cpp 
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/message_handler.cpp 
    cd $HOME/src/srrg/srrg_scan_matcher/src 
    rm laser_message_tracker.cpp 
    wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_scan_matcher/laser_message_tracker.cpp 




# Set and compile ROS packages

cd $HOME/ros/catkin_ws/src 

ln -s $HOME/src/srrg/srrg_cmake_modules . 
    ln -s $HOME/src/srrg/srrg2_orazio . 
    ln -s $HOME/src/srrg/srrg_core . 
    ln -s $HOME/src/srrg/srrg_scan_matcher . 
    ln -s $HOME/src/srrg/srrg_mapper2d . 
    ln -s $HOME/src/srrg/srrg_mapper2d_ros . 


echo "Utilizza rosdep per le dipendenze"
echo "rosdep install srrg_core"



cd $HOME/ros/catkin_ws
catkin_make -j2

# marrtino_apps

#cd $HOME/src && git clone https://bitbucket.org/iocchi/marrtino_apps.git

# rc-home-edu-learn-ros

cd $HOME/src && git clone https://github.com/robocupathomeedu/rc-home-edu-learn-ros.git



