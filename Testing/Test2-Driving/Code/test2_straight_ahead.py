#!/usr/bin/env python


"""
By C. Stubenvoll

Script for a test drive with the physical miniATV and it's digital twin in WinterSIM:
It starts the recording of a rosbag (recording all topics of the physical and digital miniATV) into a designated folder,
then waits one minute so that the physical miniATV's GNSS can get a more precise measurement of the start point (speed command = 0 m/s),
after that sends a command to both miniATV's steering topics to drive with maximum speed 
(maximum speed is definde in both miniATVs parameters, the steering topic gives a higher number that will be clipped to that individual maximum),
after 10s of driving (roughly 10m for the physical miniATV), a speed = 0 command will be sent to both miniATVs 
and another one minute wait time for the GNSS starts.
Since speed = 0 only switches off the motors and doesn't activate any breaks, both miniATVs will roll while losing speed until they stand still,
this is included in the wait time and will later be filtered out for the GNSS end position with the miniATV's status topics 
that show when the actual miniATV's speed is down to 0 again.
See the test procedure document for more information on the test setup.

The collected data is expected to give insight 
- on the similarity in movement of the physical miniATv and it's digital twin in WinterSIM 
- on delays in the current setup (f.ex. due to forwarding a topic)
- on deviations from the straight line due to wear and tear of 3D printed parts in the steering mechanism of the physical miniATV
- on the relationship between battery charge level and speed and acceleration of the physical miniATV

And the data can also be used as a benchmark
- for comparing future miniATV models in WinterSIM or in reality with it's previous versions
- comparing new miniATV (models) to their counterpart in WinterSIM/realiy 
- comparing the influence of different weather conditions

"""

import os
import subprocess
import time

import rospy
from ackermann_msgs.msg import AckermannDriveStamped


rospy.init_node('test2')

max_speed = 100 # [m/s] There will be clipping according to the config parameters of each miniATV, so this can be regarded as MAX speed
stop_speed = 0 # [m/s]
steering_angle = 0 # [deg] Straight ahead

drive_msg = AckermannDriveStamped()
test_cmd_to_pt_miniATV = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
test_cmd_to_dt_miniATV = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)


def publish_max_speed_cmd():
    rospy.loginfo("speed = MAX")
    drive_msg.drive.speed = max_speed
    drive_msg.drive.steering_angle = steering_angle
    #drive_msg.header.stamp = rospy.Time.now() # Does not work with WinterSIM/carla_ros_bridge simulation time
    test_cmd_to_dt_miniATV.publish(drive_msg)
    test_cmd_to_pt_miniATV.publish(drive_msg)


def publish_stop_cmd():
    rospy.loginfo("speed = 0")
    drive_msg.drive.speed = stop_speed
    drive_msg.drive.steering_angle = steering_angle
    #drive_msg.header.stamp = rospy.Time.now() # Does not work with WinterSIM/carla_ros_bridge simulation time
    test_cmd_to_dt_miniATV.publish(drive_msg)
    test_cmd_to_pt_miniATV.publish(drive_msg)


drive_time = time.time()
wait_after_drive = time.time()

owd = os.getcwd()
print("Orig dir: " +  str(owd))
test_start = time.time()
print("test_start type: " + str(type(test_start)))
os.chdir('/home/cs/Documents/Master_Thesis/Test_Data/Test2_Drive_Straight_Ahead')
print("Changed dir: " + str(os.getcwd()))

bag_record = subprocess.Popen(["/bin/bash", "./test2_start_rosbag_recording.sh"])

rospy.loginfo("After bash script time: ")
bash_dur = time.time() - test_start
os.chdir(owd)
rospy.loginfo(bash_dur)
gnss_wait = 60 # [s] Waittime before and after drive to get a good average for the GNSS position of the start- and endpoint
drive_duration = 10 # [s] Drive for 10 s, which equates roughly 10 m
print("After change back dir: " + str(os.getcwd()))
    
r = rospy.Rate(5)
# Can't use rospy.Time.now() instead of time.time(), or the miniATV will spawn at the wrong position (?!) 
# and the while loop will jump directly to the end - it works without WinterSIM in the laptop, it works in the miniATV - 
# so maybe it's carla's simulation time?
while not rospy.is_shutdown():
    # Before the test drive
    if time.time() - test_start <= 10:
        print("Wait before test drive!")
        print(str(time.time() - test_start) + " of 10 s")
        # wait X min at starting position before the drive for the GNSS to have a precise positon
        publish_stop_cmd()
        drive_time = time.time()
    # During the test drive
    elif time.time() - drive_time <= 10:
        print("Drive!")
        print(str(time.time() - drive_time) + " of 10 s")
        publish_max_speed_cmd() # Max speed cmd to ackermann topics!
        wait_after_drive = time.time()
    # After the test drive
    elif time.time() - drive_time > 10:
        print("Wait for test end")
        # wait again X minutes for the miniATV to stop rolling and to have a precise GNSS positon of the end position
        if time.time() - wait_after_drive <= gnss_wait:
            print("Wait after drive!")
            print(str(time.time() - wait_after_drive) + " of 10 s")
            publish_stop_cmd()
        else:
            print("END of Test2!!!!")

    else:
        rospy.loginfo("Else! => Error in code")
    print()
    r.sleep()
