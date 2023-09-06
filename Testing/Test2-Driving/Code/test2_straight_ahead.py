#!/usr/bin/env python

# By C. Stubenvoll

"""
Script for 
- comparing the current behaviour of the physical miniATv and it's digital twin in WinterSIM 
- finding delays in the current setup

And using the data aquired through this script as a benchmark
- for comparing future miniATV models in WinterSIM or in reality with it's previous versions
- comparing new miniATV (models) to their counterpart in WinterSIM/realiy 
- comparing the influence of different weather conditions


1st test:

Driving straight ahead on an even surface in the Rantavitikka testing ground (/parking lot) 
through sending a max speed command for XXXXX seconds to the ackermann topic both miniATV's listen to 
"""

import datetime
import math
import numpy
import os
import subprocess

import carla

import record_rosbag # TODO: Works???

import rospy
import rostopic
import tf
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from std_msgs.msg import String
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaStatus
from ackermann_msgs.msg import AckermannDriveStamped


#important carla topics: 

# TODO: Don't use wait or delay for 10min, because the ros topic maybe needs contact?

# TODO: Record rosbag 10s bevor and 10s after movement stopped 


max_speed = 100 # There will be clipping according to the config parameters of each miniATV
stop_speed = 0
steering_angle = 0 # Straight ahead

drive_msg = AckermannDriveStamped()
test_cmd_to_pt_miniATV = rospy.Publisher('/emergency_cmd', AckermannDriveStamped, queue_size=1)
test_cmd_to_dt_miniATV = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)

start_drive_time = rospy.get_time()
    

def publish_max_speed_cmd():
    drive_msg.drive.speed = max_speed
    drive_msg.drive.steering_angle = steering_angle
    drive_msg.header.stamp = rospy.Time.now()
    test_cmd_to_dt_miniATV.publish(drive_msg)
    test_cmd_to_pt_miniATV.publish(drive_msg)

def publish_stop_cmd():
    drive_msg.drive.speed = stop_speed
    drive_msg.drive.steering_angle = steering_angle
    drive_msg.header.stamp = rospy.Time.now()
    test_cmd_to_dt_miniATV.publish(drive_msg)
    test_cmd_to_pt_miniATV.publish(drive_msg)

    

# Go to class functions that do all the heavy lifting. Do error checking.
drive_start = 0 # fill with rostime
drive_time = 0
wait_after_drive = 0
# try:
#     test_start = rospy.get_time()
#     rosbag_record = record_rosbag()
# except rospy.ROSInterruptException:
#     rospy.loginfo("Test 2: Record Rosbag failed!!!")
#     pass
test_start = rospy.get_time()
subprocess.Popen()

gnss_wait = rospy.Duration.from_sec(60)
drive_duration = rospy.Duration.from_sec(10)
    
r = rospy.Rate(5)
while not rospy.is_shutdown(): # Loop so contact with ROS stays open during wait times #TODO: Do I need to keep contact even?
    if rospy.get_time() - test_start <= gnss_wait:
        rospy.loginfo("Start wait before test drive!")
        # wait X min at starting position before the drive for the GNSS to have a precise positon
        r.sleep()
    elif drive_time <= drive_duration:
        rospy.loginfo("Start test drive!")
        #drive_start = rospy.get_time()
        publish_max_speed_cmd() # Max speed cmd to ackermann topics!
        drive_time = rospy.get_time()
    elif drive_time > drive_duration:
        rospy.loginfo("Stop and wait for test end")
        # wait again X minutes for the miniATV to stop rolling and to have a precise GNSS positon of the end position
        publish_stop_cmd
        wait_after_drive = rospy.get_time()
        if wait_after_drive <= gnss_wait:
            publish_stop_cmd
            r.sleep
        else:
            # try:
            #     rosbag_record = rosbag_record.stop_recording_handler()
            # except rospy.ROSInterruptException:
            #     rospy.loginfo("Test 2: End rosbag recording failed!")
            #     pass
            rospy.loginfo("END of Test2!!!!")
    
    else:
        rospy.loginfo("Else! => Error in code")
        
    rospy.spin()
        
