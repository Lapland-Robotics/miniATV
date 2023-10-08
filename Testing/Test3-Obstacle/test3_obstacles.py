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


rospy.init_node('test2')

global snowflakes
snowflakes = []

max_speed = 100 # [m/s] There will be clipping according to the config parameters of each miniATV, so this can be regarded as MAX speed
stop_speed = 0 # [m/s]
steering_angle = 0 # [deg] Straight ahead
carla_max_throttle = 1.0 # The miniATVs max speed is set at 4 km/h
carla_min_throttle = 0.0
carla_no_steering = 0.0

miniATV_cmd_msg = AckermannDriveStamped()
miniATV_cmd_pub = rospy.Publisher('emergency_cmd', AckermannDriveStamped, queue_size=1)

carla_drive_msg = CarlaEgoVehicleControl()
test_cmd_to_dt_miniATV =  rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=1)

#scan_subscriber = rospy.Subscriber("/carla/ego_vehicle/lidar/scan", LaserScan, scan_callback)

# /carla/ego_vehicle/lidar/lidar1/point_cloud
# /carla/ego_vehicle/lidar/scan


def scan_callback(scan):
	print("SCAAAAAAAAAAAAAAAAAAAAAAAAAN CB!")
	stop_dist = 1.2 # [m]
	object_list = []
	msg = str("Min dist " + str(min(scan.ranges)) + " at index " + str(scan.ranges.index(min(scan.ranges))))
	#ackermann_cmd_msg = AckermannDriveStamped()
	#ackermann_pub = rospy.Publisher('emergency_cmd', AckermannDriveStamped, queue_size=1)
	print(msg)
	for deg, dist in enumerate(scan.ranges):
		#TODO: Ignore the sides and only apply the emergency stop at the front (considering the steering angles) and the back
		#TODO: Maybe: If not getting to slow with that, apply the emergency distance only in the current driving direction (and steering angle)
		

		if dist == float("inf"):
			# inf values are smaller than the min range or higher than the max range of the sensor
			# since only distances btw the min range and the stop_dist can be considered, inf values are set to higher than the stop_dist
			#rospy.loginfo(rospy.get_caller_id() + " inffff! At %s", deg)
			dist = stop_dist + 0.01
		
		if dist <= stop_dist: # scan ranges in [m]
			max_deg = len(scan.ranges)-1
			too_close_msg = str(" Too close: " + str(dist) + " at " + str(deg) + " degree!")
			if not deg == max_deg:
				next_deg = deg + 1
				next_dist = scan.ranges[next_deg]
			elif deg == max_deg:
				next_dist = scan.ranges[0]
				next_deg = 0
			debug_msg = str(" next dist = " + str(next_dist) + " deg " + str(next_deg))
			if next_dist > stop_dist: # at a range of 0.7m, obstacles have to be at least between > 2.10cm and 3.15cm
				snowflakes.append((dist, deg, rospy.get_time()))
				sf_msg = str(" Snowflake! At " + str(snowflakes[-1][1]) + " degree. "+ str(len(snowflakes)) 
					+ " snowflakes in " + str(snowflakes[-1][2]) + "[s???]")
			#TODO: elif # Only if object has to be biger than the distance between 3 degree: appends known object and doesn't create new one
			else:
				object_msg = str(" New Object!!!!")
				object_list.append((deg, next_deg)) # Later: more sophisticated would be to summarize all detected parts of an object into one



	if object_list:
		# Speed is set to 0
		print("Obstacle! Stoooooooooooooooooooooop!")
		miniATV_cmd_msg.drive.speed = 0.0
		carla_drive_msg.throttle = carla_min_throttle
		miniATV_cmd_pub.publish(miniATV_cmd_msg)
		carla_drive_msg.steer = carla_no_steering
		test_cmd_to_dt_miniATV.publish(carla_drive_msg)
	else:
		print("No obstacle: MAX speed!")
		miniATV_cmd_msg.drive.speed = speed_cmd
		miniATV_cmd_msg.drive.steering_angle = steering_cmd
		miniATV_cmd_pub.publish(miniATV_cmd_msg)
		carla_drive_msg.throttle = throttle_cmd
		carla_drive_msg.steer = steer_cmd
		test_cmd_to_dt_miniATV.publish(carla_drive_msg)


def publish_max_speed_cmd():
    # physical miniATV
    global speed_cmd
    speed_cmd = max_speed
    global steering_cmd
    steering_cmd= steering_angle
    #drive_msg.header.stamp = rospy.Time.now() # Does not work with WinterSIM/carla_ros_bridge simulation time
    #test_cmd_to_dt_miniATV.publish(drive_msg)
    #miniATV_cmd_pub.publish(miniATV_cmd_msg)
    # digital miniATV
    global throttle_cmd
    throttle_cmd = carla_max_throttle
    global steer_cmd
    steer_cmd = carla_no_steering
    #test_cmd_to_dt_miniATV.publish(carla_drive_msg)


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
    scan_subscriber = rospy.Subscriber("/carla/ego_vehicle/lidar/scan", LaserScan, scan_callback)
    if time.time() - drive_time <= drive_duration:
        print("Drive!")
        print(str(time.time() - drive_time) + " of " + str(drive_duration) + " s drive")
        publish_max_speed_cmd() # Max speed cmd to ackermann topics!
    else:
        publish_stop_cmd()
        print("END of Test3!!!!")
        sys.exit(0) # End the test script
publish_stop_cmd() # Just in case
