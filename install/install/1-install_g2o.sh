#https://github.com/RainerKuemmerle/g2o
 
sudo  apt-get update
#sudo apt-get install  libsuitesparse-dev -y
 
mkdir -p ~/lib
cd ~/lib
wget http://www.diag.uniroma1.it/iocchi/marrtino/lib/g2o-marrtino-src.tgz
tar xzf g2o-marrtino-src.tgz
rm g2o-marrtino-src.tgz 
cd g2o 
mkdir build 
cd build 
cmake ..
make -j1
 
 
sudo echo "" >> $HOME/.bashrc  && \
echo "export G2O_ROOT=\$HOME/lib/g2o" >> $HOME/.bashrc && \
echo "export G2O_BIN=\$HOME/lib/g2o/bin" >> $HOME/.bashrc && \
echo "export G2O_LIB=\$HOME/lib/g2o/lib" >> $HOME/.bashrc && \
echo "export LD_LIBRARY_PATH=\$HOME/lib/g2o/lib:\${LD_LIBRARY_PATH}" >> $HOME/.bashrc && \
echo "" >> $HOME/.bashrc



