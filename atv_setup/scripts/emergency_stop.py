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

#global ackermann_cmd_topic

global snowflakes
snowflakes = []

#global cmd_steering_angle
cmd_steering_angle = 0
#global cmd_speed
cmd_speed = 0



def scan_callback(scan):
	#global cmd_speed
	#cmd_speed = cmd.drive.speed 
	#global cmd_steering_angle
	#cmd_steering_angle = cmd.drive.steering_angle

	stop_dist = 0.7 # [m]
	msg = str("Min dist " + str(min(scan.ranges)) + " at index " + str(scan.ranges.index(min(scan.ranges))))
	#rospy.loginfo(rospy.get_caller_id() + " %s", msg)
	#rospy.loginfo(rospy.get_caller_id() + " Scan callback!") #works
	#data.ranges is of type tuple! len(data.ranges) = 360
	object_list = [] #Currently not needed
	ackermann_cmd_msg = AckermannDriveStamped()
	ackermann_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
	for deg, dist in enumerate(scan.ranges):
		#TODO: Ignore the sides and only apply the emergency stop at the front (considering the steering angles) and the back
		#TODO: Maybe: If not getting to slow with that, apply the emergency distance only in the current driving direction (and steering angle)
		#TODO if parts of the robot itself

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
			#rospy.loginfo(rospy.get_caller_id() + "%s", (debug_msg))
			if next_dist > stop_dist: # at a range of 0.6m, obstacles have to be at least between > 2.10cm and 3.15cm
				snowflakes.append((dist, deg, rospy.get_time()))
				sf_msg = str(" Snowflake! At " + str(snowflakes[-1][1]) + " degree. "+ str(len(snowflakes)) 
					+ " snowflakes in " + str(snowflakes[-1][2]) + "[s???]")
				#rospy.loginfo(rospy.get_caller_id() + "%s", sf_msg)
			#TODO: elif # Only if object has to be biger than the distance between 3 degree: appends known object and doesn't create new one
			else:
				object_msg = str(" New Object!!!!")
				#rospy.loginfo(rospy.get_caller_id() + object_msg)
				object_list.append((deg, next_deg)) # Later: more sophisticated would be to summarize all detected parts of an object into one



	if object_list:
		# Speed is set to 0
		rospy.loginfo(rospy.get_caller_id() + " Stoooop!")
		ackermann_cmd_msg.drive.speed = 0.0
		ackermann_pub.publish(ackermann_cmd_msg)
	else:
		rospy.loginfo(rospy.get_caller_id() + " cmd_speed %s", cmd_speed)
		ackermann_cmd_msg.drive.speed = cmd_speed #cmd.drive.speed 
		#rospy.loginfo(rospy.get_caller_id() + " cmd_speed %s", cmd.drive.speed)
		ackermann_cmd_msg.drive.steering_angle = cmd_steering_angle #cmd.drive.steering_angle
		ackermann_pub.publish(ackermann_cmd_msg)

			
			#talker()

def emergency_cmd_callback(cmd):
	rospy.loginfo(rospy.get_caller_id() + " em_cmd callback!")
	global cmd_speed
	cmd_speed = cmd.drive.speed 
	global cmd_steering_angle
	cmd_steering_angle = cmd.drive.steering_angle
	#rospy.loginfo(rospy.get_caller_id() + " cmd_speed %s", cmd_speed) #works

"""+
angle_min: -3.14159274101
angle_max: 3.14159274101
angle_increment: 0.0174532923847
=> With the angle increment and the index of the "min tuple" in ranges, we can also see where the nearest obstacle is!
"""

def pcl2_callback(data):
	rospy.loginfo(rospy.get_caller_id() + " PCl2 callback! ")
	rospy.loginfo(rospy.get_caller_id() + " height %s", data.height) 
    	rospy.loginfo(rospy.get_caller_id() + " width %s", data.width) 
	rospy.loginfo(rospy.get_caller_id() + " point_step %s", data.point_step)
	rospy.loginfo(rospy.get_caller_id() + " len data %s", len(data.data))


def scan_listener():

	#rospy.loginfo(rospy.get_caller_id() + " Scan Listener!")
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.Subscriber("emergency_cmd", AckermannDriveStamped, emergency_cmd_callback)
	#rospy.Subscriber("ouster/points", PointCloud2, pcl2_callback)

    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

def pcl2_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
	#rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("ouster/points", PointCloud2, pcl2_callback)

    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

	

def talker():
	pub = rospy.Publisher('emergency_stop_debug', String, queue_size=10)
	#rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		stop_str = "Stoooop! %s" % rospy.get_time()
		#rospy.loginfo(stop_str)
		pub.publish(stop_str)
		rate.sleep()


"""
# Get min dist from pointcloud2
ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
min_pcl2 = #meters? ToDo: Maybe change metrics



# Compare to our "emergency distance"
lidar_min_range = 0.5 #meters

emergency_dist = lidar_min_range + 0.1 #meters

if min_pcl2 <= emergency_dist:
	#Send stop signal via ros topics to ackermann_cmd

	#for test purposes: Send "Stooop!" to stop topic
"""
rospy.init_node('emergency_stop')

r = rospy.Rate(5.0)
while not rospy.is_shutdown():
	#emergency_cmd_listener()
	#listener()
	scan_listener()
	
	#pcl2_listener()
	r.sleep()
