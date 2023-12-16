#!/usr/bin/env python

"""
By C. Stubenvoll

Computes the average GNSS position and average accuracy out of a rosbag with GNSS data under the topic /atv_gps. Saves this into a csv file.

Maybe relevant:
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


class AverageGNSS():

    def __init__(self):
        self.gnss_topic_name = '/atv_gps'
        csv_filename = "evaluation_test1.csv"

        owd = os.getcwd()
        test1_data_dir = '/home/cs/Documents/Master_Thesis/Test_Data/Test1_Alignment'
        test2_data_dir = '/home/cs/Documents/Master_Thesis/Test_Data/Test2_Drive_Straight_Ahead'
        self.data_dir = test1_data_dir
        os.chdir(self.data_dir) # operate in the directory where the bagfiles are stored!

        self.bag_files = [f for f in os.listdir(self.data_dir) if f.endswith('.bag')]
        self.bag_files = sorted(self.bag_files)
        print(self.bag_files)

        csv_header = ['bagfile', 'average latitude', 'average longitude', 'average altitude', 'average horizontal accuracy [m]', 'average vertical accuracy[m]']
        with open(csv_filename, 'w') as csvfile:
            csv_writer = csv.writer(csvfile) # creating a csv writer object
            csv_writer.writerow(csv_header)
            for bag in self.bag_files:
                av_lat, av_lon, av_alt, av_hor_acc, av_vert_acc = self.msg_average(bag)
                csv_row = [bag, av_lat, av_lon, av_alt, av_hor_acc, av_vert_acc]
                csv_writer.writerow(csv_row)
            #self.get_rosbag_info(bag)
            #self.get_rosbag_topics(bag)

    def msg_average(self, bag_name):
        print("rosbag messages for ")
        print(bag_name)
        bag = rosbag.Bag(bag_name)
        latitudes = []
        longitudes = []
        altitudes = []
        hor_accs = []
        vert_accs = []
        for topic, msg, t in bag.read_messages(topics=['/atv_gps']):

            #print(msg.position_covariance[8])
            latitudes.append(msg.latitude)
            longitudes.append(msg.longitude)
            altitudes.append(msg.altitude)
            hor_accs.append(msg.position_covariance[0])
            vert_accs.append(msg.position_covariance[8])

        bag.close()
        av_lat = sum(latitudes)/len(latitudes)
        av_lon = sum(longitudes)/len(longitudes)
        av_alt = sum(altitudes)/len(altitudes)
        av_hor_acc = sum(hor_accs)/len(hor_accs)
        av_vert_acc = sum(vert_accs)/len(vert_accs)
        #print(av_lat)
        #print(av_lon)
        #print(av_alt)
        #print(av_hor_acc)
        #print(av_vert_acc)
        return av_lat, av_lon, av_alt, av_hor_acc, av_vert_acc
