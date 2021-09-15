#!/bin/bash
echo "Start RPLidar_a3 bash"
ls -l /dev |grep ttyUSB
sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros_a3 rplidar.launch 
echo "Start RPLidar_a3 bash END"
