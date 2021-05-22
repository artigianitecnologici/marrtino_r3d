#!/bin/bash
date
#export ROBOT_NAME=marrwy
#export ROBOT_TYPE=marrtinox
export MARRTINO_R3D_HOME=$HOME/src/marrtino_r3d
#roscore &
cd bringup
python wsbringup.py &
