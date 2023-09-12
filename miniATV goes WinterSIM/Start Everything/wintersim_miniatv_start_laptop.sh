#!/bin/bash

#export ROS_MASTER_URI=http://cs-worklaptop:11311
sleep 5s
# Start everything on the Laptop/PC side:
# First, make this file executable!
# source /opt/carla-ros-bridge/melodic/setup.bash has to be added to the bash.rc beforehand!

# Starts in Laptop:
#	- WinterSIM
#	- carla_ros_bridge (and with that a roscore)
#	- carla_ackermann_control
#	- multimaster_fkie (first discover the miniATV roscore, then sync)


gnome-terminal --tab -e "WinterSim1.2_rantavitikka/CarlaUE4.sh" 
sleep 5s #Wait for the WinterSIM world to be ready
gnome-terminal --tab -e "roslaunch carla_ros_bridge gps_carla_ros_bridge_w_ego_vehicle.launch"
# multimaster_fkie communication between two seperate rosmasters
# You have to setup the multimaster according to my Multimaster_fkie Documentation.odt or add the setup commands here!
sleep 6s

export ROS_MASTER_URI=http://cs-worklaptop:11311

sleep 2s
gnome-terminal --tab -e "node_manager"
gnome-terminal --tab -e "rosrun master_discovery_fkie master_discovery _robot_hosts:=[ATV]"
sleep 6s
gnome-terminal --tab -e "rosrun master_sync_fkie master_sync"
