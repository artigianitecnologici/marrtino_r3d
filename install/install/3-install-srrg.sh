sudo apt update
sudo apt install libwebsockets-dev
cd $HOME/src
git clone https://github.com/LCAS/spqrel_navigation.git

mkdir -p $HOME/src/srrg
cd $HOME/src/srrg
cp $HOME/src/marrtino_r3d/install/srrg.tar.gz .
#wget http://www.diag.uniroma1.it/iocchi/marrtino/lib/g2o-marrtino-src.tgz
tar xzf srrg.tar.gz
#
cd $HOME/ros/catkin_ws/src
    #ln -s $HOME/src/srrg/srrg2_navigation .
    ln -s $HOME/src/srrg/srrg_cmake_modules .
    ln -s $HOME/src/srrg/srrg2_orazio . 
    ln -s $HOME/src/srrg/srrg_core . 
    ln -s $HOME/src/srrg/srrg_scan_matcher . 
    ln -s $HOME/src/srrg/srrg_mapper2d . 
    ln -s $HOME/src/srrg/srrg_mapper2d_ros . 
    ln -s $HOME/src/spqrel_navigation .
cd $HOME/ros/catkin_ws
catkin_make -j1