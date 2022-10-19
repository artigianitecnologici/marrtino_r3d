
# apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

 apt-get update 
 apt-get install -y libsuitesparse-dev ros-melodic-gmapping 
   
# g2o

mkdir -p $HOME/lib
    cd $HOME/lib 
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/lib/g2o-marrtino-src.tgz 
    tar xzvf g2o-marrtino-src.tgz 
    cd g2o 
    mkdir -p build 
    cd build 
    cmake .. 
    make 

#     rm g2o-marrtino-src.tgz 

 echo "" >> $HOME/.bashrc  
    echo "export G2O_ROOT=\$HOME/lib/g2o" >> $HOME/.bashrc  
    echo "export G2O_BIN=\$HOME/lib/g2o/bin" >> $HOME/.bashrc  
    echo "export G2O_LIB=\$HOME/lib/g2o/lib" >> $HOME/.bashrc  
    echo "export LD_LIBRARY_PATH=\$HOME/lib/g2o/lib:\$LD_LIBRARY_PATH" >> $HOME/.bashrc  
    echo "" >> $HOME/.bashrc


# srrg packages
 mkdir -p $HOME/src/srrg 
    cd $HOME/src/srrg  
    git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git 
    git clone https://gitlab.com/srrg-software/srrg_core.git  
    git clone https://gitlab.com/srrg-software/srrg_core_ros.git  
    git clone https://gitlab.com/srrg-software/srrg_scan_matcher.git  
    git clone https://gitlab.com/srrg-software/srrg_mapper2d.git  
    git clone https://gitlab.com/srrg-software/srrg_mapper2d_ros.git
    git clone https://gitlab.com/srrg-software/srrg2_navigation.git


# Patches
 cd $HOME/src/srrg/srrg_cmake_modules   
    git checkout 8e023a96e1d1fe07572db698c22ca3741c0bb06c  
    cd $HOME/src/srrg/srrg_core   
    git checkout 3d3310c70da18077c90ecc61256fa7ea34029517  
    cd src && rm CMakeLists.txt  
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/patches/srrg_core/src/CMakeLists.txt  
    cd $HOME/src/srrg/srrg_core_ros   
    git checkout 9ccfdad65d1e453b905a1fe9b4467ed3c4e89251  
    cd src && rm CMakeLists.txt  
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/patches/srrg_core_ros/src/CMakeLists.txt  
    cd $HOME/src/srrg/srrg_mapper2d_ros  
    rm CMakeLists.txt package.xml   
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/patches/srrg_mapper2d_ros/CMakeLists.txt  
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/patches/srrg_mapper2d_ros/package.xml   
    cd src   
    rm srrg_mapper2d_node.cpp   
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/patches/srrg_mapper2d_ros/srrg_mapper2d_node.cpp  
    rm message_handler.cpp  
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/patches/srrg_mapper2d_ros/message_handler.cpp  
    cd $HOME/src/srrg/srrg_scan_matcher   
    cd src   
    rm laser_message_tracker.cpp   
    wget http://www.diag.uniroma1.it/publiciocchi/marrtino/patches/srrg_scan_matcher/laser_message_tracker.cpp


# Link
 cd $HOME/ros/catkin_ws/src  
    ln -s $HOME/src/srrg/srrg_cmake_modules .   
    ln -s $HOME/src/srrg/srrg_core .   
    ln -s $HOME/src/srrg/srrg_core_ros .   
    ln -s $HOME/src/srrg/srrg_scan_matcher .  
    ln -s $HOME/src/srrg/srrg_mapper2d .  
    ln -s $HOME/src/srrg/srrg_mapper2d_ros .
    ln -s $HOME/src/srrg/srrg2_navigation


# ROS packages

 if [ "$MACHTYPE" = "aarch64" ]; then \
       /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make -j2" ; \
    elif [ "$MACHTYPE" = "armv7l" ]; then \
       /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make -j1" ; \
    else \
       /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make" ; \
    fi

 
