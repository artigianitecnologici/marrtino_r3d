# marrtino_r3d
# MARRTINO implementazioni
# by Ferrarini Fabio - ROBOTICS-3D.com

# Installazione da Zer0


# 1 - Costruzione della Mappa con dati gmapping 
    
##    a1.Esecuzione di gmapping ( marrtino)
    
      cd $HOME/src/marrtino_apps/mapping
      roslaunch gmapping.launch

       ---- CONSIGLIATO ---- 
##    a2.Esecuzione di srrg_mapping ( marrtino)
       cd $HOME/src/marrtino_apps/mapping
       roslaunch srrg_mapper.launch

    
##    b.Registrare /tf /scan con rosbag ( client)

      export ROS_IP=`hostname -I`
      export ROS_MASTER_URI=http://10.3.1.1:11311 
    
      rosbag record -O risto2.bag /scan /tf
      ctrl+c per interrompere

##    c.Aprire rviz sul client
      
      export ROS_IP=`hostname -I`
      export ROS_MASTER_URI=http://10.3.1.1:11311 
      cd src/marrtino_r3d 

##    d.Salvare la mappa ( marrtino)
      rosrun map_server map_saver -f nomemappa

##    e.Visualizzare la mappa
      eom nomemappa.pgm


# 2- navigation

   a. cd $HOME/src/marrtino_r3d/launch
      roslaunch amcl.launch map_name:=casa


   b. cd $MARRTINO_APPS_HOME/navigation
      roslaunch move_base.launch


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

### 3. MAPPING 


### 4. NAVIGATION





### 6. SIMULATION



## Acknowledgments

## progetti studio 

git clone https://github.com/FabioScap/marrtino_utilities
git clone https://github.com/RBinsonB/Nox_robot.git
git clone https://github.com/artigianitecnologici/script.git

# ROS-ROBOTICS-by-Examples
 https://github.com/PacktPublishing/ROS-Robotics-By-Example
 git clone https://github.com/PacktPublishing/ROS-Robotics-By-Example.git

## configuration .bashrc


# CONFIGURAZIONE PERSONALIZZATA 

source /opt/ros/melodic/setup.bash
source /home/ubuntu/ros/catkin_ws/devel/setup.bash

export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps
export ROBOT_TYPE=stage
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

## flash firmware orazio
 cd $HOME/src/srrg/srrg2_orazio/srrg2_orazio/firmware_build/atmega2560
 make clean
 make
 make orazio.hex

## conf. firmware orazio
  cd script
  ./orazio_config.sh

  sul browse  localhost:9000



