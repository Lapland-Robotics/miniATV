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


rospy.init_node('laptop_emergency_stop')

global snowflakes
snowflakes = []

# global variables
steering_cmd = 0
speed_cmd = 0
throttle_cmd = 0.0
steer_cmd = 0.0

# Forwarding to those topics in case of no obstacle:
miniATV_cmd_msg = AckermannDriveStamped()
miniATV_cmd_pub = rospy.Publisher('emergency_cmd', AckermannDriveStamped, queue_size=1)
carla_drive_msg = CarlaEgoVehicleControl()
test_cmd_to_dt_miniATV =  rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=1)


def scan_callback(scan):
    print("SCAAAAAAAAAAAAAAAAAAAAAAAAAN CB!")
    stop_dist = 1.5 # [m]
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
        miniATV_cmd_msg.drive.steering_angle = 0.0
        carla_drive_msg.throttle = 0.0
        carla_drive_msg.steer = 0.0
        miniATV_cmd_pub.publish(miniATV_cmd_msg)
        test_cmd_to_dt_miniATV.publish(carla_drive_msg)
    else:
        print("No obstacle: MAX speed!")
        global speed_cmd
        print("Speed cmd " + str(speed_cmd))
        miniATV_cmd_msg.drive.speed = speed_cmd
        miniATV_cmd_msg.drive.steering_angle = steering_cmd
        miniATV_cmd_pub.publish(miniATV_cmd_msg)
        carla_drive_msg.throttle = throttle_cmd
        carla_drive_msg.steer = steer_cmd
        test_cmd_to_dt_miniATV.publish(carla_drive_msg)
        
def cb_laptop_emergency_cmd(cmd):
    global speed_cmd
    speed_cmd = cmd.drive.speed 
    global steering_cmd
    steering_cmd = cmd.drive.steering_angle
    
def cb_vehicle_emergency_cmd(cmd):
    global throttle_cmd
    throttle_cmd = cmd.throttle
    global steer_cmd
    steer_cmd = cmd.steer

    

def scan_listener():
    print("Scan listener")
    rospy.Subscriber("/carla/ego_vehicle/lidar/scan", LaserScan, scan_callback)
    rospy.Subscriber("laptop_emergency_cmd", AckermannDriveStamped, cb_laptop_emergency_cmd)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_emergency_cmd", CarlaEgoVehicleControl, cb_vehicle_emergency_cmd)

    rospy.spin()

r = rospy.Rate(10.0) # 10 Hz loop rate for rospy
while not rospy.is_shutdown():
    scan_listener()
    r.sleep()
