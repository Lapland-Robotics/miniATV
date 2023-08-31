#!/bin/bash
# run this file:  bash wintersim_miniatv_start.sh 

gnome-terminal --window -e "bash 'wintersim_miniatv_start_laptop.sh'"

gnome-terminal --window -e "bash 'wintersim_miniatv_start_miniatv.sh'"

# needed: ip_adresses.sh with all the credentials for starting the miniATV

# Starts in Laptop:
#	- WinterSIM
#	- carla_ros_bridge (and with that a roscore with use_sim_time = True)
#	- carla_ackermann_control
#	- multimaster_fkie (first discover the miniATV roscore, then sync)

# Starts in miniATV:
#	- roscore (with use_sim_time = False)
#	- multimaster_fkie (first discover the Laptop roscore, then sync)
#	- launch the desired ros launchfile 



