#!/bin/bash
# start everything for the WinterSIM on the MiniATV side
# export ROS_MASTER_URI=http://ATV:11311 has to be added to the bashrc 
# make this file executable!
# and the basic settings for the multimaster_fkies have to be set beforehand (see my Documentation on that)

# needed: ip_adresses.sh with the credentials for the miniATV

# Starts in miniATV:
#	- roscore (with use_sim_time = False)
#	- multimaster_fkie (first discover the Laptop roscore, then sync)
#	- launch the desired ros launchfile 


source ip_addresses.sh
echo "MiniATV startup"

sshpass -p $atv_password ssh -X -tty $atv_username@$robot_IP << endoflist
sleep 4s
source ~/.bashrc
sleep 4s

roscore &
sleep 6s
rosrun master_discovery_fkie master_discovery _robot_hosts:=[cs-worklaptop] &
sleep 4s
rosrun master_sync_fkie master_sync &
roslaunch atv_setup emergency_stop.launch &
endoflist

# Alternative:
# roslaunch atv_setup basic_wintersim_test.launch &
# roslaunch atv_setup two_rosserial_nodes.launch &
sleep 4s





