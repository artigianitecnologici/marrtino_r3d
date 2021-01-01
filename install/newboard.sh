#!/bin/bash
#
# 11/12/2019
# New motor board, srrg2_orazio firmware
#


cd ~/src/srrg/
git clone https://gitlab.com/srrg-software/srrg2_orazio.git
cd srrg2_orazio

# compile firmware
cd  ~/src/srrg/srrg2_orazio/srrg2_orazio/firmware_build/atmega2560
make 

cd $HOME/ros/catkin_ws
catkin_make -j1

