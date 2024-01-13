#!/usr/bin/env python

"""
By C. Stubenvoll

Traces delays in the digital twin design by finding the rosbag timestamps when the commands arrived at certain topics.
Saves this into a csv file.

Thanks to:
https://www.geeksforgeeks.org/writing-csv-files-in-python/
http://wiki.ros.org/rosbag/Code%20API
"""
import subprocess #
import yaml
import os
import rospy #
import rosbag
from rosbag.bag import Bag
import csv
import re
import numpy
import time
#from geopy import distance


class AverageGNSS():

    def __init__(self):
        self.gnss_topic_name = '/atv_gps'
        csv_filename = "averages_gnss_test1.csv"

        owd = os.getcwd()
        test2_rosbag_info = '/home/cs/Documents/Master_Thesis/Test_Data/Test2_Drive_Straight_Ahead/rosbags_cmd_topics_overview.csv'
        test2_delays = '/home/cs/Documents/Master_Thesis/Test_Data/Test2_Drive_Straight_Ahead/delays_cmd_topics.csv'
        test2_data_dir = '/home/cs/Documents/Master_Thesis/Test_Data/Test2_Drive_Straight_Ahead'
        self.data_dir = test2_data_dir
        os.chdir(self.data_dir) # operate in the directory where the bagfiles are stored!

        self.bag_files = [f for f in os.listdir(self.data_dir) if f.endswith('.bag')]
        self.bag_files = sorted(self.bag_files)
        bag_info_header = ['rosbag name','start', 'end', 'duration', '# msgs in rosbag', 'vehicle_control_cmd', 'frequency', '# msgs', 'emergency_cmd', 'frequency', '# msgs', 'ackermann_cmd', 'frequency', '# msgs']
        bag_info_rows = []
        rows_short = []
        for bag in self.bag_files:
            print("Name: " + str(bag))
            header_long, row_long, header_short, row_short = self.print_msgs(bag)
            rows_short.append(row_short)
            #name, duration, num_rosbag_msg, vehicle_control_cmd_name, vehicle_control_cmd_freq, vehicle_control_cmd_nmsgs, emergency_cmd_name, emergency_cmd_freq, emergency_cmd_nmsgs, ackermann_cmd_name, ackermann_cmd_freq, ackermann_cmd_nmsgs = self.get_rosbag_info(bag)
            bag_info_rows.append(self.get_rosbag_info(bag))

        with open(test2_rosbag_info, 'w') as csvfile:
            csv_writer = csv.writer(csvfile) # creating a csv writer object
            csv_writer.writerow(bag_info_header)
            csv_writer.writerows(bag_info_rows)
            csvfile.close()

        
        with open(test2_delays, 'w') as csvfile:
            csv_writer = csv.writer(csvfile) # creating a csv writer object
            csv_writer.writerow(header_short)
            csv_writer.writerows(rows_short)
            csvfile.close()

            
    def print_msgs(self, bag_name):
        # get start time, end time and duration for csv file
        info_dict = yaml.load(Bag(bag_name, 'r')._get_yaml_info())
        start = info_dict.get('start')
        end = info_dict.get('end')
        duration = info_dict.get('duration')
        topics_list = info_dict.get('topics')

        bag = rosbag.Bag(bag_name)
        num = 0
        t_init = rospy.Duration(0)
        dt_prev_throttle = 0
        t_dt_max_speed_cmd = 0
        t_dt_zero_speed_cmd = 0

        pt_prev_em_cmd = 0
        t_pt_em_zero_cmd = 0
        t_pt_em_max_cmd = 0
        
        pt_prev_ack_cmd = 0
        t_pt_ack_zero_cmd = 0
        t_pt_ack_max_cmd = 0

        prev_dt_throttle_control = 0
        t_throttle_control_start = 0
        t_throttle_control_stop = 0

        prev_dt_velocity = 0
        t_dt_velocity_start = 0
        t_dt_velocity_stop = 0
        dt_highest_vel = 0

        prev_pt_speed = 0
        t_pt_speed_start = 0
        t_pt_speed_stop = 0
        pt_highest_speed = 0
        counter = 0
        # t is the time the message got recorded! Not the source time of the message
        for topic, msg, t in bag.read_messages(topics=['/carla/ego_vehicle/vehicle_control_cmd', '/emergency_cmd', '/ackermann_cmd', '/atv_state', '/carla/ego_vehicle/vehicle_info', '/carla/ego_vehicle/vehicle_status']): # other: '/atv_gps' , '/master_discovery/linkstats'
            if num == 0 and topic in ['/carla/ego_vehicle/vehicle_control_cmd', '/emergency_cmd', '/ackermann_cmd']:
                init_t = t
                #print(msg)
                #print(init_t-start)

            if topic == '/carla/ego_vehicle/vehicle_control_cmd':
                if msg.throttle != 0 and dt_prev_throttle == 0:
                    t_dt_max_speed_cmd = t.to_sec()
                    #print("dt throttle -> max")
                    #print(t_dt_max_speed_cmd)
                    #print((t_dt_max_speed_cmd-t_init))
                if msg.throttle == 0 and dt_prev_throttle != 0:
                    t_dt_zero_speed_cmd = t.to_sec()
                    #print("dt throttle -> zero")
                    #print(t_dt_zero_speed_cmd)
                    #print((t_dt_zero_speed_cmd-t_dt_max_speed_cmd))
                dt_prev_throttle = msg.throttle

            if topic == '/emergency_cmd':
                if msg.drive.speed != 0 and pt_prev_em_cmd == 0:
                    t_pt_em_max_cmd = t.to_sec()
                    #print("pt_em -> max")
                    #print(t_pt_em_max_cmd)
                    #print((t_pt_em_max_cmd-t_init))
                if msg.drive.speed == 0 and pt_prev_em_cmd != 0:
                    t_pt_em_zero_cmd = t.to_sec()
                    #print("pt_em -> zero")
                    #print(t_pt_em_zero_cmd)
                    #print((t_pt_em_zero_cmd-t_pt_em_max_cmd))
                    #print(t_pt_em_zero_cmd. - start)
                pt_prev_em_cmd = msg.drive.speed

            if topic == '/ackermann_cmd':
                if msg.drive.speed != 0 and pt_prev_ack_cmd == 0:
                    t_pt_ack_max_cmd = t.to_sec()
                    #print("pt_ack -> max")
                    #print(t_pt_ack_max_cmd)
                    #print((t_pt_ack_max_cmd-t_init))
                    
                if msg.drive.speed == 0 and pt_prev_ack_cmd != 0:
                    t_pt_ack_zero_cmd = t.to_sec()
                    #print("pt_ack -> zero")
                    #print(t_pt_ack_zero_cmd)
                    #print((t_pt_ack_zero_cmd-t_pt_ack_max_cmd))
                pt_prev_ack_cmd = msg.drive.speed

            if topic in ['/carla/ego_vehicle/vehicle_control_cmd', '/emergency_cmd', '/ackermann_cmd']:
                num += 1
                #print("NUM: " + str(num))

            if topic == '/atv_state':
                if msg.drive.speed > 6 and prev_pt_speed < 6:
                    t_pt_speed_start = t.to_sec()
                    prev_counter = counter
                    counter += 1
                    speed = msg.drive.speed
                    if prev_counter < counter:
                        print("physical twin speed change to " + str(speed) + " from previous " + str(prev_pt_speed))
                if msg.drive.speed < 6 and prev_pt_speed > 6:
                    t_pt_speed_stop = t.to_sec()
                if msg.drive.speed > pt_highest_speed:
                    pt_highest_speed = msg.drive.speed
                prev_pt_speed = msg.drive.speed
            
            if topic == '/carla/ego_vehicle/vehicle_status':
                if msg.control.throttle != 0 and prev_dt_throttle_control == 0:
                    t_throttle_control_start = t.to_sec()
                if msg.control.throttle == 0 and prev_dt_throttle_control != 0:
                    t_throttle_control_stop = t.to_sec()
                
                if msg.velocity != 0 and prev_dt_velocity == 0:
                    t_dt_velocity_start = t.to_sec()
                    print("dt velocity start " + str(t_dt_velocity_start))
                if msg.velocity == 0 and prev_dt_velocity != 0:
                    t_dt_velocity_stop = t.to_sec()
                    print("dt_velocity stops " + str(t_dt_velocity_stop))
                if msg.velocity > dt_highest_vel:
                    dt_highest_vel = msg.velocity

                prev_dt_velocity = msg.velocity
                prev_dt_throttle_control = msg.control.throttle

            

        t_first_cmd = t_init.to_sec() - start #[s]
        # How does the cmds propagate through the command topics
        diff_dt_pt_em_max = t_pt_em_max_cmd - t_dt_max_speed_cmd #[s]
        diff_pt_em_ack_max = t_pt_ack_max_cmd - t_pt_em_max_cmd
        diff_pt_em_ack_zero = t_pt_ack_zero_cmd - t_pt_em_zero_cmd
        diff_dt_pt_em_zero = t_pt_em_zero_cmd - t_dt_zero_speed_cmd #[s]
        # When are the cmds received and the execution starts
        diff_throttle_max = t_throttle_control_start - t_dt_max_speed_cmd
        diff_throttle_zero = t_throttle_control_stop - t_dt_zero_speed_cmd
        diff_dt_vel_start = t_dt_velocity_start - t_throttle_control_start
        diff_dt_vel_stop = t_dt_velocity_stop - t_throttle_control_stop
        #diff_dt_vel_start = t_dt_velocity_start - t_dt_max_speed_cmd
        #diff_dt_vel_stop = t_dt_velocity_stop - t_dt_zero_speed_cmd
        diff_pt_speed_start = t_pt_speed_start - t_pt_ack_max_cmd
        diff_pt_speed_stop = t_pt_speed_stop - t_pt_ack_zero_cmd
        

        row  =[bag_name, duration, start, t_first_cmd, t_dt_max_speed_cmd, diff_dt_pt_em_max, t_pt_em_max_cmd, diff_pt_em_ack_max, t_pt_ack_max_cmd, t_dt_zero_speed_cmd, diff_dt_pt_em_max, t_pt_em_zero_cmd, diff_pt_em_ack_zero, t_pt_ack_zero_cmd, end]
        row_short = [bag_name, diff_throttle_max, diff_dt_vel_start, diff_dt_pt_em_max, diff_pt_em_ack_max, diff_pt_speed_start, diff_throttle_zero, diff_dt_vel_stop, diff_dt_pt_em_zero, diff_pt_em_ack_zero, diff_pt_speed_stop]
            #print(topic)
            #print(msg)
            #print(t)
            #print(t.to_sec())
        print("Highest dt vel ", dt_highest_vel)
        print("Highest pt speed ", pt_highest_speed)
        header = ["Bag", "Duration", "Start", "Time of first cmd [s]", "Time of digital twin max speed cmd [s]", "Time difference to max speed cmd for physical twin [s]", "Time of physical twin max speed cmd [s]", "Time between input and output max speed cmd for emergency stop [s]", "Time of output max speed cmd of emergency stop in physical twin [s]", "Time of digital twin zero speed cmd [s]", "Time difference to zero speed cmd for physical twin [s]", "Time of physical twin zero speed cmd [s]", "Time between input and output zero speed cmd for emergency stop [s]", "Time of output zero speed cmd of emergency stop in physical twin [s]", "end [s]"]
        header_short = ["Bag", "Diff max speed cmd vehicle_control_cmd & dt.status.throttle_cmd", "Diff max speed cmd dt.status.throttle_cmd & digital twin starts driving", "Diff max speed cmd vehicle_control_cmd & emergency_cmd [s]", "Diff max speed cmd emergency_cmd & ackermann_cmd [s]", "Diff max speed cmd ackermann_cmd & physical twin starts driving [s]", "Diff zero speed cmd vehicle_control_cmd & dt.status.throttle_cmd [s]", "Diff zero speed cmd dt.status.throttle_cmd & digital twin stops [s]", "Diff zero speed cmd vehicle_control_cmd & emergency_cmd [s]", "Diff zero speed cmd emergency_cmd & ackermann_cmd [s]", "Diff zero speed cmd ackermann_cmd & physical twin stops [s]"]
        
        return header, row, header_short, row_short
        


    def get_rosbag_info(self, bag_name):
        # Source: http://wiki.ros.org/rosbag/Cookbook
        info_dict = yaml.load(Bag(bag_name, 'r')._get_yaml_info())
        #print("rosbag info for ")
        name = info_dict.get('path')
        #print('Name' + str(name))
        start = info_dict.get('start')
        #print('Start ' + str(start))
        end = info_dict.get('end')
        #print('End ' + str(end))
        duration = info_dict.get('duration')
        #print('Duration ' + str(duration))
        num_rosbag_msg = info_dict.get('messages')
        #print('Messages ' + str(num_rosbag_msg))
        #print('topics')
        topics_list = info_dict.get('topics')
        #print(topics_list)
        topics=['/carla/ego_vehicle/vehicle_control_cmd', '/emergency_cmd', '/ackermann_cmd']
        topics_info = []
        for t in topics:
            topic = filter(lambda topics_list: topics_list['topic'] == t, topics_list)
            if topic:
                topic = topic[0]
                topic_name = topic.get('topic')
                #print("topic name " + str(topic_name))
                topic_freq = topic.get('frequency')
                #print("frequency " + str(topic_freq))
                topic_num_msgs = topic.get('messages')
                #print("number msgs " + str(topic_num_msgs))
                topics_info.append([topic_name, topic_freq, topic_num_msgs])
            else:
                topics_info.append([None, None, None])
        return name, start, end, duration, num_rosbag_msg, topics_info[0][0], topics_info[0][1], topics_info[0][2], topics_info[1][0], topics_info[1][1], topics_info[1][2], topics_info[2][0], topics_info[2][1], topics_info[2][2]


    def get_rosbag_topics(self, bag_name):
        # Source: http://wiki.ros.org/rosbag/Cookbook
        bag = rosbag.Bag(bag_name)
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for val in bag.get_type_and_topic_info()[1].values():
            types.append(val[0])
        print("for rosbag " + str(bag_name))
        print("rosbag topics " + str(topics))
        print("rosbag types " + str(types))


if __name__ == "__main__":
    AverageGNSS()
