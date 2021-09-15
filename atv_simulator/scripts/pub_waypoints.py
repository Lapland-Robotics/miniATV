#!/usr/bin/env python

#idea from: Thanks Oscar Lima from https://answers.ros.org/question/203782/rviz-marker-line_strip-is-not-displayed/
#Edited HK 

import os
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math

rospy.init_node('waypoint_publisher')
pub_waypoints_lines = rospy.Publisher('~waypoints_pub', MarkerArray, queue_size=1)
rospy.loginfo('Publishing Waypoints')

FILE = os.path.expanduser("~") + '/catkin_ws/src/atv_simulator/waypoints/waypoints.csv'

coordinateCount = 0
marker_ID_counter = 0
f = file (FILE,"r")
rawCoordinates = f.readlines()
f.close()
coordinates=[]
for i in rawCoordinates:
	#print i
	coordinates.append([float(i.split(",")[0]), float(i.split(",")[1])])

#print coordinates
coordinateCount = len(coordinates)
print "Waypoint count", coordinateCount

markerArray = MarkerArray()

for i in range(coordinateCount):
    marker = Marker()
    marker.id = marker_ID_counter
    marker.header.frame_id = "/map"
    #marker.lifetime = rospy.Duration()
    
    marker.type = marker.SPHERE 
    marker.action = marker.ADD
   # marker scale
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
   # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
   # marker orientaiton
    #marker.pose.orientation.x = 0.0
    #marker.pose.orientation.y = 0.0
    #marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
   # marker position
    marker.pose.position.x = coordinates [i][0]
    marker.pose.position.y = coordinates [i][1]
    marker.pose.position.z = 0.0
    marker_ID_counter += 1
    markerArray.markers.append(marker)

marker_line = Marker()
marker_line.id = marker_ID_counter
marker_line.header.frame_id= "/map"
marker_line.type = marker.LINE_STRIP 
marker_line.action = marker.ADD
# marker scale
marker_line.scale.x = 0.03
# marker color
marker_line.color.a = 1.0
marker_line.color.b = 1.0
marker_line.color.r = 1.0
#  marker_line.color.g = 0.0
# marker orientaiton
marker_line.pose.orientation.w = 1.0
# marker position
marker_line.pose.position.x = 0.0
marker_line.pose.position.y = 0.0
marker_line.pose.position.z = 0.0
# marker line points
marker_line.points = []
for i in range(coordinateCount):
    p = Point ()
    p.x = coordinates [i][0]
    p.y = coordinates [i][1]
    p.z = 0
    marker_line.points.append(p)
markerArray.markers.append(marker_line) 
#add linestrip to markerArray

  # first point
  #  first_line_point = Point()
  #  first_line_point.x = coordinates [i][0]
  #  first_line_point.y = coordinates [i][1]
  #  first_line_point.z = 0.0
  #  marker.points.append(first_line_point)
    # second point
  #  second_line_point = Point()
  #  second_line_point.x = coordinates [i+1][0]
  #  second_line_point.y = coordinates [i+1][1]
   # second_line_point.z = 0.0
   # marker.points.append(second_line_point)
    
  #  marker_ID_counter += 1


while not rospy.is_shutdown():
  pub_waypoints_lines.publish(markerArray)
  rospy.sleep(2)


