#!/usr/bin/env bash
echo "********* HECTOR SLAM INSTALLATION **********"
echo "*********************************************"
cd $HOME/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
sudo -H apt-get install -y libqt4-dev
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/hector_slam .
cd ..
catkin_make
cd ..
echo "************* END INSTALLATION **************"
echo "*********************************************"

