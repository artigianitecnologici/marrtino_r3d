echo "****************************"
echo "**** MARRTINO R3D START ****"
echo "****************************"
sudo ntpdate ntp.ubuntu.com
echo "start ..."
roscore &
sleep 2
roslaunch marrtino_r3d bringup.launch
#sleep 40
