#!/bin/bash
# run this file: bash run_atv_robot_remote1.sh
# at end of line "&" is terminal parameter to run command in background
source ip_addresses.sh
echo "sending waypoint command"
# need install sshpass
#sshpass -p $hoppop ssh -o StrictHostKeyChecking=no $my_username@$robot_IP
sshpass -p $hoppop ssh -tty $my_username@$robot_IP << endoflist
source ~/.bashrc
rosrun atv_setup waypoint_navigation.py
endoflist

