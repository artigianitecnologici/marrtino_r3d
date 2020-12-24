cd $HOME/src
git clone https://github.com/ros-perception/image_pipeline
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/image_pipeline .
cd ..
catkin_make -j1
