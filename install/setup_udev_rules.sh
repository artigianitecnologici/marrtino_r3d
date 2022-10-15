#!/bin/bash -e

echo "Setting-up permissions for RealSense devices"

exec 3>&2
exec 2> /dev/null
con_dev=$(ls /dev/video* | wc -l)
exec 2>&3

if [ $con_dev -ne 0 ];
then
	echo -e "\e[32m"
	read -p "Remove all RealSense cameras attached. Hit any key when ready"
	echo -e "\e[0m"
fi

sudo cp rules/rplidar.rules /etc/udev/rules.d/
sudo cp rules/80-marrtino.rules /etc/udev/rules.d/
sudo cp rules/99-realsense-libusb.rules /etc/udev/rules.d/
sudo cp rules/ldlidar.rules /etc/udev/rules.d/
#sudo udevadm control --reload-rules && udevadm trigger

#setting access device
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyACM0

echo "udev-rules successfully installed"
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "