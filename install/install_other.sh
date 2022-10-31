cd $HOME/src 
git clone https://github.com/artigianitecnologici/talk.git
git clone https://github.com/artigianitecnologici/ros_waypoint_generator.git
git clone https://github.com/artigianitecnologici/waypoint_navigation.git
git clone https://github.com/artigianitecnologici/marrtinox_description.git
git clone https://github.com/artigianitecnologici/marrwy_description.git
git clone https://github.com/FabioScap/marrtino_utilities
git clone https://github.com/LCAS/spqrel_navigation.git
#git clone https://bitbucket.org/iocchi/laser_analysis.git
#git clone https://bitbucket.org/iocchi/rococo_navigation.git
#sudo apt-get install libsvm-dev -y
#git clone https://github.com/spencer-project/spencer_people_tracking.git

# Link
cd $HOME/ros/catkin_ws/src
      ln -s $HOME/src/talk .
      ln -s $HOME/src/waypoint_navigation .
      ln -s $HOME/src/marrtinox_description .
      ln -s $HOME/src/marrwy_description .
      ln -s $HOME/src/marrtino_utilities .
      ln -s $HOME/src/spqrel_navigation .
      #ln -s $HOME/src/laser_analysis . 
      #ln -s $HOME/src/rococo_navigation . 
      #ln -s $HOME/src/spencer_people_tracking . 
     
cd $HOME/ros/catkin_ws
catkin_make -j1

