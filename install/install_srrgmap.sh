# SRRG mapper


 if [ ! -d $HOME/src/srrg/srrg_mapper2d ]; then

      cd $HOME/src/srrg
      git clone https://gitlab.com/srrg-software/srrg_core.git
      git clone https://gitlab.com/srrg-software/srrg_scan_matcher.git
      git clone https://gitlab.com/srrg-software/srrg_mapper2d.git
      git clone https://gitlab.com/srrg-software/srrg_mapper2d_ros.git
	  git clone https://gitlab.com/srrg-software/srrg_joystick_teleop.git

      cd srrg_mapper2d_ros
      rm CMakeLists.txt package.xml
      wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/CMakeLists.txt
      wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/package.xml
      cd src
      rm srrg_mapper2d_node.cpp
      wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/srrg_mapper2d_node.cpp
      rm message_handler.cpp
      wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_mapper2d_ros/message_handler.cpp

      cd ../..
      cd srrg_scan_matcher
      cd src
      rm laser_message_tracker.cpp
      wget http://www.diag.uniroma1.it/iocchi/marrtino/patches/srrg_scan_matcher/laser_message_tracker.cpp



      # Link
      cd $HOME/ros/catkin_ws/src
      ln -s $HOME/src/srrg/srrg_core .
      ln -s $HOME/src/srrg/srrg_scan_matcher .
      ln -s $HOME/src/srrg/srrg_mapper2d .
      ln -s $HOME/src/srrg/srrg_mapper2d_ros .
	  ln -s $HOME/src/srrg/srrg_joystick_teleop .
	  
      

    fi
cd $HOME/ros/catkin_ws
catkin_make -j1
 
#Manca la libreria srrg_core (https://gitlab.com/srrg-software/srrg_core/tree/master)


