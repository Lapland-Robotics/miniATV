#!/bin/bash
# start the recording of the rosbag, since the terminal is occupied until this is done and doesn't
# execute commands meanwhile, e.g. steering commands

echo "start recording of the rosbag"

rosbag record --duration=2m --all # 2m = 2 minutes TODO: change to appropriate timespan
