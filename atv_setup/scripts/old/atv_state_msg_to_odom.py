#!/usr/bin/env python
# Copy from https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
# and modified
# This script Subscribe ATV_State from robot controller (ESP32) and convert state to odom
# atv_state = Measures Steering Angle(float32, radians), Measured Speed (m/s)

import rospy, math
from math import sin, cos, pi, tan, sqrt
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped
import sys

v = 0
angleZ = 0
arc = 0
irc = 0

def cmd_callback(data):
  global angleZ
  global v
  angleZ = data.drive.steering_angle
  v = data.drive.speed
  #print "v =",v
  #print "angleZ =",angleZ


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
odom_broadcaster = tf.TransformBroadcaster()
rospy.Subscriber("atv_state",AckermannDriveStamped, cmd_callback, queue_size=5)


x = rospy.get_param('~initial_pose_x',0)
y = rospy.get_param('~initial_pose_y',0)
#th = rospy.get_param('~initial_pose_a',0)
th = 0

vx = 0.0
vy = 0.0
vth = 0.0


current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(5.0)
while not rospy.is_shutdown():
    wheelbase = rospy.get_param('~wheelbase', 1.0)
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    vx = v
    vy = 0
    vth = v*tan(angleZ)/wheelbase

    delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    delta_th = vth * dt;
    
    print "dx =",delta_x
    print "dy =",delta_y
    print "dth =",delta_th
    x += delta_x
    y += delta_y
    th += delta_th
    #print "x =",x
    #print "y =",y

    # since all odometry is 6DOF :) :) :) we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
 
    # set the velocity
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))


    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
