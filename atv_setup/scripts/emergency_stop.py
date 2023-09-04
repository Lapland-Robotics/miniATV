#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry #Maybe not needed
import message_filters

from ackermann_msgs.msg import AckermannDriveStamped
import sys
# Emergency stop if lidar data shows that something is closer than x cm
# Ouster Lidar: min range is 0.5 m, but the measurements show that measures until 0.42m occured
# Ouster Lidar: measured values smaller than the min range and higher than the max range are both "inf"
# Ouster Lidar: 0 (or 360 degree) are at the back where the power cable connects and increase counter clockwise 
# object list is only needed if the objects are used for intelligent driving or when objects have to be bigger than 3 degree at the stop_dist


global snowflakes
snowflakes = []

cmd_steering_angle = 0
cmd_speed = 0



def scan_callback(scan):
	stop_dist = 0.7 # [m]
	msg = str("Min dist " + str(min(scan.ranges)) + " at index " + str(scan.ranges.index(min(scan.ranges))))
	object_list = [] #Currently not needed
	ackermann_cmd_msg = AckermannDriveStamped()
	ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
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
		rospy.loginfo(rospy.get_caller_id() + " Stoooop!")
		ackermann_cmd_msg.drive.speed = 0.0
		ackermann_pub.publish(ackermann_cmd_msg)
	else:
		rospy.loginfo(rospy.get_caller_id() + " cmd_speed %s", cmd_speed)
		ackermann_cmd_msg.drive.speed = cmd_speed
		ackermann_cmd_msg.drive.steering_angle = cmd_steering_angle
		ackermann_pub.publish(ackermann_cmd_msg)


def emergency_cmd_callback(cmd):
	rospy.loginfo(rospy.get_caller_id() + " em_cmd callback!")
	global cmd_speed
	cmd_speed = cmd.drive.speed 
	global cmd_steering_angle
	cmd_steering_angle = cmd.drive.steering_angle


# angle_min: -3.14159274101
# angle_max: 3.14159274101
# angle_increment: 0.0174532923847
# => With the angle increment and the index of the "min tuple" in ranges, we can also see where the nearest obstacle is!


def scan_listener():
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.Subscriber("emergency_cmd", AckermannDriveStamped, emergency_cmd_callback)
    	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


rospy.init_node('emergency_stop')

r = rospy.Rate(10.0) # 10 Hz loop rate for rospy
while not rospy.is_shutdown():
	scan_listener()
	r.sleep()
