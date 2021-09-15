#!/usr/bin/env python

# Copy from https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
# and modified
# This script convert Twist to Ackermann command and Published it and Subscribe ATV_State
# from robot controller (ESP32) and convert state to odom
# atv_state = Measures Steering Angle(float32, radians), Measured Speed (m/s)

import rospy, math
from math import sin, cos, tan, sqrt
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import sys

def twist_cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global odom_frame_id
  global ackermann_pub
  
  #speed_x = data.linear.x
  if data.angular.z == 0 or data.linear.x == 0:
    steering = 0
  else :
    steering = data.angular.z
    # BELOW MATH IS NOT NEEDED, BECAUSE WE USE teb_local_planner AND IT CONVERT TWIST COMMAND TO
    # STEERING ANGLE WHEN parameter ~<name>/cmd_angle_instead_rotvel is true (set also parameter "wheelbase")
    # http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
    #radius = data.linear.x / data.angular.z
    #steering math.atan(wheelbase / radius)
    #steering = math.atan(radius) #Speed from front/steering wheels
  #print steering
  steering_msg = AckermannDriveStamped()
  steering_msg.header.stamp = rospy.Time.now()
  steering_msg.header.frame_id = odom_frame_id
  #steering_msg.drive.steering_angle = steering
  steering_msg.drive.steering_angle = steering #*0.8
  steering_msg.drive.speed = data.linear.x
  # Publish Ackermann
  ackermann_pub.publish(steering_msg)


v = 0
angleZ = 0
arc = 0
irc = 0

def atv_state_cmd_callback(data):
  global angleZ
  global v
  angleZ = data.drive.steering_angle #*0.8
  v = data.drive.speed*1.2
  #print "v =",v
  #print "angleZ =",angleZ

rospy.init_node('atv_control_and_odom')

twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
odom_frame_id = rospy.get_param('/amcl/odom_frame_id', 'odom') 
wheelbase = rospy.get_param('/move_base/TebLocalPlannerROS/wheelbase', 1.0)

odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
odom_broadcaster = tf.TransformBroadcaster()
ackermann_pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
rospy.Subscriber("atv_state",AckermannDriveStamped, atv_state_cmd_callback, queue_size=1)
rospy.Subscriber(twist_cmd_topic, Twist, twist_cmd_callback, queue_size=1)

x = rospy.get_param('~initial_pose_x',0)
y = rospy.get_param('~initial_pose_y',0)
#th = rospy.get_param('~initial_pose_a',0)
th = 0

vx = 0.0
vy = 0.0
vth = 0.0

rospy.loginfo("Node 'atv_control_and_odom' started.\nListening to %s,%s, publishing to %s,%s. Frame id: %s, wheelbase: %f", twist_cmd_topic,"/atv_state","/odom" ,ackermann_cmd_topic, odom_frame_id, wheelbase)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(5.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    vx = v
    vy = 0
    vth = v*tan(angleZ)/wheelbase

    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt
    #print wheelbase
    #print "dx =",delta_x
    #print "dy =",delta_y
    #print "dth =",delta_th
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


