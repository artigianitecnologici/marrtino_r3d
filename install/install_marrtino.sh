cd $HOME/src
git clone https://bitbucket.org/iocchi/marrtino_apps.git
git clone https://github.com/Slamtec/rplidar_ros.git
git clone https://bitbucket.org/iocchi/stage_environments.git


cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/rplidar_ros/ .
ln -s $HOME/src/stage_environments/ .
cd $HOME/ros/catkin_ws
catkin_make -j1

