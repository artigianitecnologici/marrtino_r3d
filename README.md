# marrtino_r3d
# MARRTINO implementazioni
# by Ferrarini Fabio - ROBOTICS-3D.com

# Installazione da Zer0


# bringup
  roslaunch bringup.launch 
  - robot.launch
  - teleop.launch
  - rplidar.launch

# SRRG
srrg_mapper2d 
srrg_mapper2d_ros
srrg_scan_matcher
spqrel_navigation
srrg2_orazio 
srrg_cmake_modules
srrg_core               
srrg_joystick_teleop    


# spqrel_navigation
      git clone https://github.com/LCAS/spqrel_navigation.git
 
# Auto Exploration 
      https://developer.qualcomm.com/project/auto-exploration-navigation

      git clone https://github.com/hrnr/m-explore.git
      cd $HOME/src/m-explore
      git branch melodic-devel remotes/origin/melodic-devel


# 1 - Costruzione della Mappa con dati gmapping 
    
##    Esecuzione di gmapping ( marrtino)
    
      cd $HOME/src/marrtino_apps/mapping
      roslaunch gmapping.launch

       ---- CONSIGLIATO ---- 
##    Esecuzione di srrg_mapping ( marrtino)
       cd $HOME/src/marrtino_apps/mapping
       roslaunch srrg_mapper.launch

    
##    Registrare /tf /scan con rosbag ( client)

      rosbag record -O casa.bag /scan /tf /odom
      ctrl+c per interrompere

##    risprodurre /tf /scan con rosbag ( client)
      rosparam set use_sim_time true
      rosbag play casa.bag --clock

      rosbag play imu.bag -d 2 /imu:=/testimu --clock 
 
      -d nsec (wait n sencond )
      
      --hz=HZ

##    c.Aprire rviz sul client
      
      export ROS_IP=`hostname -I`
      export ROS_MASTER_URI=http://10.3.1.1:11311 
      export ROS_HOSTNAME=10.3.1.17
      cd $HOME/src/marrtino_r3d/launch


      rosrun rviz rviz
      
      roslaunch marrtino_r3d marrtinox_rviz_navigation.launch



##    d.Salvare la mappa ( marrtino)
      cd $HOME/src/marrtino_r3d/maps
      rosrun map_server map_saver -f  casa  

##    e.Visualizzare la mappa
      eom nomemappa.pgm


# 2a - navigation with amcl & move_base

   a. cd $HOME/src/marrtino_r3d/launch
      roslaunch amcl.launch map_name:=casa


   b. cd $MARRTINO_APPS_HOME/navigation
      roslaunch move_base.launch

# 2b - navigation with srrg_localizer (amcl ) &  spqrel_planner_node ( move_base)

   a. cd $HOME/src/marrtino_r3d/launch
      roslaunch srrg_localizer.launch map_name:=casa

  c.Aprire rviz sul client
      
      export ROS_IP=`hostname -I`
      export ROS_MASTER_URI=http://10.3.1.1:11311 
      cd src/marrtino_r3d/launch

   c. $cd src/marrtino



cd $MARRTINO_APPS_HOME/mapping
rosrun rviz rviz -d mapping.rviz

NAVIGATION

cd $MARRTINO_APPS_HOME/navigation
roslaunch amcl.launch mapsdir:=$HOME/playground map_name:=office
cd $MARRTINO_APPS_HOME/navigation
roslaunch move_base.launch  

roslaunch move_base_gbn.launch

### 2. Start the odometry and motor control
cd $HOME/src/marrtino_rd3/launch
./bringup.sh


### 3. Avvio  ambiente di simulazione

./simulation.sh


### WAYPOINT

1 . Generate waypoint
``` 
    rosrun ros_waypoint_generator ros_waypoint_generator_custom
```



### 3. MAPPING 


### 4. NAVIGATION





### 6. SIMULATION



## apt update: signatures were invalid: F42ED6FBAB17C654

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -



## progetti studio 

git clone https://github.com/FabioScap/marrtino_utilities
git clone https://github.com/RBinsonB/Nox_robot.git
git clone https://github.com/artigianitecnologici/script.git

git clone https://github.com/Thedush/ros_waypoint_generator.git
git https://github.com/AriYu/ros_waypoint_generator

git clone https://github.com/zshn25/turtlesim_cleaner.git
# ROS-ROBOTICS-by-Examples
 https://github.com/PacktPublishing/ROS-Robotics-By-Example
 git clone https://github.com/PacktPublishing/ROS-Robotics-By-Example.git

## configuration .bashrc


# CONFIGURAZIONE PERSONALIZZATA 

source /opt/ros/melodic/setup.bash
source /home/ubuntu/ros/catkin_ws/devel/setup.bash


export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps
export ROBOT_TYPE=stage
export ROBOT_NAME=marrtinox
export MARRTINO_VERSION=4.0.1


export G2O_ROOT=$HOME/lib/g2o
export G2O_BIN=$HOME/lib/g2o/bin
export G2O_LIB=$HOME/lib/g2o/lib
export LD_LIBRARY_PATH=$HOME/lib/g2o/lib:${LD_LIBRARY_PATH}



### FAQ
- enable autologin
  open /usr/share/lightdm/lightdm.conf.d/60-lightdm-gtk-greeter.conf
  aggiungi
  autologin-user=ubuntu
  
- install openssh
  sudo apt update
  sudo apt install openssh-server
  sudo ufw allow ssh  

## flash firmware orazio
 cd $HOME/src/srrg/srrg2_orazio/srrg2_orazio/firmware_build/atmega2560
 make clean
 make
 make orazio.hex

## conf. firmware orazio
  cd script
  ./orazio_config.sh

  sul browse  localhost:9000

# NAVIGAZIONE TRAMITE WAYPOINT


rostopic pub /load_paths std_msgs/String "data: '$HOME/src/marrtino_r3d/navigation/counter_fwd1.csv'"
rosrun ros_waypoint_generator ui_nav_waypoints.py $HOME/src/marrtino_r3d/navigation/

# AGGIORNAMENTO GIT
git add *
git commit -m "Messaggio per la commit"
git push 

# installare google chrom
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo dpkg -i google-chrome-stable_current_amd64.deb

# one
## two 
### three
sudo apt install ros-melodic-dwb-local-planner
sudo apt install  ros-melodic-dwa-local-planner



# Configure www
cd /var/www/html
sudo ln -s $HOME/src/marrtino_r3d/www/bringup .