
# installazione 
https://github.com/acrobotic/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435
#

sudo apt update
sudo apt-get install ros-melodic-realsense2-camera
sudo apt-get install ros-melodic-realsense2-description
echo "install rules"

rs-enumerate-devices
Device info:
    Name                          :     Intel RealSense T265
    Serial Number                 :     11622110117
    Firmware Version              :     0.2.0.951
    Physical Port                 :     2-1-2
    Product Id                    :     0B37
    Usb Type Descriptor           :     3.1
    Product Line                  :     T200

Stream Profiles supported by Tracking Module
 Supported modes:
    stream       resolution      fps       format
    Fisheye 1     848x800       @ 30Hz     Y8
    Fisheye 2     848x800       @ 30Hz     Y8
    Gyro         N/A            @ 200Hz    MOTION_XYZ32F
    Accel        N/A            @ 62Hz     MOTION_XYZ32F
    Pose         N/A            @ 200Hz    6DOF

