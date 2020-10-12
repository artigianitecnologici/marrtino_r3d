# marrtino_r3d
# MARRTINO implementazioni
# by Ferrarini Fabio - ROBOTICS-3D.com



1-  Costruzione della Mappa con dati gmapping 
    
    a.Esecuzione di gmapping ( marrtino)
    
      $roslaunch marrtino_r3d gmapping.launch
    
    b.Registrare /tf /scan con rosbag ( client)

      $export ROS_IP=`hostname -I`
      $export ROS_MASTER_URI=http://10.3.1.1:11311 
    
      $rosbag record -O casa.bag /scan /tf
      ctrl+c per interrompere

    c.Salvare la mappa ( marrtino)
    
      rosrun map_server map_saver -f "office”
    
    d.Visualizzare la mappa
      eom office.pgm


2- navigation
   cd $MARRTINO_APPS_HOME/navigation
   roslaunch amcl.launch mapsdir:=$HOME/playground map_name:=casaodom
--------
cd $MARRTINO_APPS_HOME/navigation
roslaunch move_base.launch

cd $MARRTINO_APPS_HOME/mapping
rosrun rviz rviz -d mapping.rviz

NAVIGATION

cd $MARRTINO_APPS_HOME/navigation
roslaunch amcl.launch mapsdir:=$HOME/playground
cd $MARRTINO_APPS_HOME/navigation
roslaunch move_base.launch
map_name:=mymap

   
