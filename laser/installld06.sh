# install lidar ld06
sudo apt update
sudo apt install libudev-dev
cd $HOME/src
git clone https://github.com/AlessioMorale/ld06_lidar.git
cs
ln -s $HOME/src/ld06_lidar .
cm