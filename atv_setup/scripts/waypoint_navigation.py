#!/usr/bin/env python

# Origin: https://github.com/cristian-frincu/navifgation_waypoints_scripts
# Edited: HK

FILE = '/home/hk/catkin_ws/src/atv_setup/waypoints/waypoints.csv'
#FILE = '/home/hk/catkin_ws/src/waypoints/waypoints/waypoints.csv'
import rospy
import actionlib

#move_base_msgs
from move_base_msgs.msg import *
from geometry_msgs.msg import *

def simple_move(x,y,z,w):


    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    #create goal
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()
    #send goal
    sac.send_goal(goal)
    print "Sending goal:",x,y,z,w
    #finish
    sac.wait_for_result()
    #print result
    print sac.get_result()

def talker(coordinates):
    #publishing as PoseArray, as a reference for user
    f = open(FILE,'r')
    array = PoseArray()
    array.header.frame_id = 'map'
    array.header.stamp = rospy.Time.now()

    for goal in f:
        coordinates = goal.split(",")
        pose = Pose()
        pose.position.x = float(coordinates[0])
        pose.position.y = float(coordinates[1])
        pose.orientation.z = float(coordinates[2])
        pose.orientation.w = float(coordinates[3])
        array.poses.append(pose)

        pub = rospy.Publisher('simpleNavPoses', PoseArray, queue_size=100)
        rate = rospy.Rate(1) # 1hz

    count = 0
    while count<1:
		rate.sleep()	
		pub.publish(array)
		count +=1


if __name__ == '__main__':
    try:
        rospy.init_node('simple_move')

        #sending commands for the robot to travel
        f = open(FILE,'r')
        for goal in f:
			coordinates = goal.split(",")
			simple_move(float(coordinates[0]),float(coordinates[1]),float(coordinates[2]),float(coordinates[3]))
			talker(coordinates)
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"


