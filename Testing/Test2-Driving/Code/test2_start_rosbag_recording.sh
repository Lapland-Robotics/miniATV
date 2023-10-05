#!/bin/bash
# start the recording of the rosbag, since the terminal is occupied until this is done and doesn't
# execute commands meanwhile, e.g. steering commands

echo "start recording of the rosbag"

rosbag record --all /topic __name:=test2_bag # records with that node name so it can be ended later
