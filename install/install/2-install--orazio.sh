

sudo apt update 
sudo     apt install -y \
        tmux less sudo nano wget iputils-ping net-tools htop \
        arduino arduino-mk libreadline-dev libwebsockets-dev libeigen3-dev \
        ros-noetic-control-msgs ros-noetic-tf ros-noetic-tf2 
   

# Init ROS workspace

mkdir -p $HOME/ros/catkin_ws/src

mkdir -p ~/ros/catkin_ws/src
cd ~/ros/catkin_ws/
catkin_make



# orazio ROS node

mkdir -p $HOME/src/srrg 
cd $HOME/src/srrg 
git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git
git clone https://gitlab.com/srrg-software/srrg2_orazio.git

# thin_state_publisher (with patch)

cd $HOME/src/ 
git clone https://bitbucket.org/ggrisetti/thin_drivers.git 

cp $HOME/install/thin_state_publisher.patch /$HOME/src/thin_drivers/thin_state_publisher.patch 

cd $HOME/src/thin_drivers 
git apply thin_state_publisher.patch


# Config .bash 

echo "export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps" >> $HOME/.bashrc

echo "export "export ROBOT_TYPE=marrtino" >> $HOME/.bashrc
echo "export "export MARRTINO_VERSION=4.0.1" >> $HOME/.bashrc
echo "export "export ROBOT_NAME=marrtinox" >> $HOME/.bashrc
echo "export "export ROS_MASTER_URI=http://10.3.1.100:11311" >> $HOME/.bashrc
echo "export "export ROS_HOSTNAME=10.3.1.100" >> $HOME/.bashrc
echo "set -g mouse on" > $HOME/.tmux.conf" 

touch ~/.sudo_as_admin_successful


# Set and compile ROS packages
cd $HOME/ros/catkin_ws/src 
    ln -s $HOME/src/srrg/srrg_cmake_modules . 
    ln -s $HOME/src/srrg/srrg2_orazio .
    ln -s $HOME/src/thin_drivers/thin_state_publisher 
 cd $HOME/ros/catkin_ws
 catkin_make -j1

# marrtino_apps



#cd $HOME/src/srrg/srrg_cmake_modules 
#    git checkout 6d0236de9a99e44e76584095d51bb4f2574c77dc

#cd $HOME/src/srrg/srrg2_orazio 
#    git checkout 64ecae3304dd3da5bbcf3f1a16fcdf2f399a3671
#cd $HOME/ros/catkin_ws
 #catkin_make -j1
