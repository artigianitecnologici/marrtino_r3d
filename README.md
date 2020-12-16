# marrtino_r3d
# MARRTINO implementazioni
# by Ferrarini Fabio - ROBOTICS-3D.com



1-  Costruzione della Mappa con dati gmapping 
    
    a1.Esecuzione di gmapping ( marrtino)
    
      cd $HOME/src/marrtino_apps/mapping
      roslaunch gmapping.launch

       ---- CONSIGLIATO ---- 
    a2.Esecuzione di srrg_mapping ( marrtino)
       cd $HOME/src/marrtino_apps/mapping
       roslaunch srrg_mapper.launch

    
    b.Registrare /tf /scan con rosbag ( client)

      export ROS_IP=`hostname -I`
      export ROS_MASTER_URI=http://10.3.1.1:11311 
    
      rosbag record -O casa.bag /scan /tf
      ctrl+c per interrompere

    c.Aprire rviz sul client
      
      export ROS_IP=`hostname -I`
      export ROS_MASTER_URI=http://10.3.1.1:11311 
      cd src/marrtino_r3d 

    d.Salvare la mappa ( marrtino)
    
      rosrun map_server map_saver -f office
    
    e.Visualizzare la mappa
      eom office.pgm


2- navigation

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
roslaunch amcl.launch mapsdir:=$HOME/playground
cd $MARRTINO_APPS_HOME/navigation
roslaunch move_base.launch map_name:=mymap

   
