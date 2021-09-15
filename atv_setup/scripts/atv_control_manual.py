#!/usr/bin/env python
# Run on terminals: 
# - roscore
# - rosrun rosserial_python serial_node.py _port:=/dev/ATV_Interface_Port _baud:=57600
# - rosrun atv_setup atv_control_manual.py

import rospy, math
from math import sin, cos, tan, sqrt
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import sys

def cmd_callback():
  global ackermann_cmd_topic
  global odom_frame_id
  global ackermann_pubs
  steering_msg = AckermannDriveStamped()
  steering_msg.header.stamp = rospy.Time.now()
  steering_msg.header.frame_id = "odom"
  steering_msg.drive.steering_angle = 0.2
  steering_msg.drive.speed = 0.0
  # Publish Ackermann
  ackermann_pub.publish(steering_msg)

def atv_state_cmd_callback(data):
  global angleZ
  global v
  angleZ = data.drive.steering_angle
  v = data.drive.speed
  print "v =",v
  print "angleZ =",angleZ


rospy.init_node('atv_control_manual')

ackermann_cmd_topic = '/ackermann_cmd'
ackermann_pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
rospy.Subscriber("atv_state",AckermannDriveStamped, atv_state_cmd_callback, queue_size=1)

r = rospy.Rate(5.0)
while not rospy.is_shutdown():
    cmd_callback()
    r.sleep()
