#!/usr/bin/env python


"""
By C. Stubenvoll
"""
import os
import sys
import subprocess
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import LaserScan


rospy.init_node('test3')


max_speed = 100 # [m/s] There will be clipping according to the config parameters of each miniATV, so this can be regarded as MAX speed
stop_speed = 0 # [m/s]
steering_angle = 0 # [deg] Straight ahead
carla_max_throttle = 1.0 # The miniATVs max speed is set at 4 km/h
carla_min_throttle = 0.0
carla_no_steering = 0.0



miniATV_cmd_msg = AckermannDriveStamped()
miniATV_cmd_pub = rospy.Publisher('laptop_emergency_cmd', AckermannDriveStamped, queue_size=1)

carla_drive_msg = CarlaEgoVehicleControl()
test_cmd_to_dt_miniATV =  rospy.Publisher('/carla/ego_vehicle/vehicle_emergency_cmd', CarlaEgoVehicleControl, queue_size=1)

# /carla/ego_vehicle/lidar/lidar1/point_cloud
# /carla/ego_vehicle/lidar/scan


def publish_max_speed_cmd():
    # physical miniATV
    miniATV_cmd_msg.drive.speed = max_speed
    miniATV_cmd_msg.drive.steering_angle = steering_angle
    #drive_msg.header.stamp = rospy.Time.now() # Does not work with WinterSIM/carla_ros_bridge simulation time
    miniATV_cmd_pub.publish(miniATV_cmd_msg)
    # digital miniATV
    carla_drive_msg.throttle = carla_max_throttle
    carla_drive_msg.steer = carla_no_steering
    test_cmd_to_dt_miniATV.publish(carla_drive_msg)


def publish_stop_cmd():
    print('Zerooooooo speed')
	# Zero speed command doesn't have to be checked for safety
    # physical miniATV
    miniATV_cmd_msg.drive.speed = stop_speed
    miniATV_cmd_msg.drive.steering_angle = steering_angle
    #drive_msg.header.stamp = rospy.Time.now() # Does not work with WinterSIM/carla_ros_bridge simulation time
    miniATV_cmd_pub.publish(miniATV_cmd_msg)
    # digital miniATV
    carla_drive_msg.throttle = carla_min_throttle
    carla_drive_msg.steer = carla_no_steering
    test_cmd_to_dt_miniATV.publish(carla_drive_msg)


    
drive_duration = 10 # [s] Drive for 10 s, which equates roughly 10 m

r = rospy.Rate(5)
# Can't use rospy.Time.now() instead of time.time(), or the miniATV will spawn at the wrong position (?!) 
# and the while loop will jump directly to the end - it works without WinterSIM in the laptop, it works in the miniATV - 
# so maybe it's carla's simulation time?
#scan_subscriber = rospy.Subscriber("/carla/ego_vehicle/lidar/scan", LaserScan, scan_callback)

drive_time = time.time()
publish_stop_cmd() # Just in case
while not rospy.is_shutdown():
    if time.time() - drive_time <= drive_duration:
        print("Drive!")
        print(str(time.time() - drive_time) + " of " + str(drive_duration) + " s drive")
        publish_max_speed_cmd() # Max speed cmd to ackermann topics!
    else:
        publish_stop_cmd()
        print("END of Test3!!!!")
        sys.exit(0) # End the test script
publish_stop_cmd() # Just in case
