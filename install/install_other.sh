cd $HOME/src
git clone https://github.com/artigianitecnologici/talk.git
git clone https://github.com/artigianitecnologici/ros_waypoint_generator.git
git clone https://github.com/artigianitecnologici/marrtinox_description.git
git clone https://github.com/artigianitecnologici/marrwy_description.git

# Link
cd $HOME/ros/catkin_ws/src
      ln -s $HOME/src/talk .
      ln -s $HOME/src/ros_waypoint_generator .
      ln -s $HOME/src/marrtinox_description .
      ln -s $HOME/src/marrwy_description .
      

     
cd $HOME/ros/catkin_ws
catkin_make -j1

