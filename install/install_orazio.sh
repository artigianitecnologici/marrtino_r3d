sudo apt update
sudo apt install libwebsockets-dev
mkdir -p $HOME/src/srrg
cd $HOME/src/srrg
git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git
git clone https://gitlab.com/srrg-software/srrg2_orazio.git 
cd $HOME/ros/catkin_ws/src
ln -s $HOME/src/srrg/srrg_cmake_modules .
ln -s $HOME/src/srrg/srrg2_orazio .
cd $HOME/ros/catkin_ws
catkin_make -j1

