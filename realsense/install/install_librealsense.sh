cd $HOME/src
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir  build  && cd build
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
make -j1
sudo make install
cd $HOME/src/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
echo "Apply the change (needs to be run by root):"
sudo su
udevadm control --reload-rules && udevadm trigger