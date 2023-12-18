#!/usr/bin/env python

"""
By C. Stubenvoll

Computes the average GNSS position and average accuracy out of a rosbag with GNSS data under the topic /atv_gps. Saves this into a csv file.

Thanks to:
https://www.geeksforgeeks.org/writing-csv-files-in-python/
http://wiki.ros.org/rosbag/Code%20API
https://www.kdnuggets.com/2017/02/removing-outliers-standard-deviation-python.html
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
#from geopy import distance


class OutlierreducedGNSS():

    def __init__(self):
        self.gnss_topic_name = '/atv_gps'
        csv_filename = "averages_gnss_test1.csv"

        owd = os.getcwd()
        test1_data_dir = '/home/cs/Documents/Master_Thesis/Test_Data/Test1_Alignment'
        test1_output = '/home/cs/Documents/Master_Thesis/Test_Data/Test1_Alignment/outlierreduced_gnss_test1.csv'
        test1_ref_ponts = '/home/cs/Documents/Master_Thesis/Test_Data/Test1_Alignment/reference_points_both_xyz_latlongalt.csv'
        test2_data_dir = '/home/cs/Documents/Master_Thesis/Test_Data/Test2_Drive_Straight_Ahead'
        self.data_dir = test1_data_dir
        os.chdir(self.data_dir) # operate in the directory where the bagfiles are stored!

        self.bag_files = [f for f in os.listdir(self.data_dir) if f.endswith('.bag')]
        self.bag_files = sorted(self.bag_files)

        csv_header = ['bagfile', 'outlierreduced mean latitude', 'outlierreduced mead longitude', 'outlierreduced mean altitude', 'mean latitude', 'mean longitude', 'mean altitude', 'sd latitude', 'sd longitude', 'sd altitude', 'number of datapoints', 'percentage outliers']
        with open(test1_averages, 'w') as csvfile:
            csv_writer = csv.writer(csvfile) # creating a csv writer object
            csv_writer.writerow(csv_header)
            for bag in self.bag_files:
                lat_mean_outlierreduced, lon_mean_outlierreduced, alt_mean_outlierreduced, lat_mean, lon_mean, alt_mean, lat_sd, lon_sd, alt_sd, num_datapoints, data_points_left = self.outlier_correction(bag)
                csv_row = [bag, lat_mean_outlierreduced, lon_mean_outlierreduced, alt_mean_outlierreduced, lat_mean, lon_mean, alt_mean, lat_sd, lon_sd, alt_sd, num_datapoints, data_points_left]
                csv_writer.writerow(csv_row)
            csvfile.close()

        
    
def outlier_correction(self, bag_name):
        print("rosbag messages for ")
        print(bag_name)
        bag = rosbag.Bag(bag_name)
        # stopre the lat, lon, alt together, since I have to throw out whole points instead of a single value of points that doesn't fit
        gnss_points = []
        latitudes = []
        longitudes = []
        altitudes = []
        for topic, msg, t in bag.read_messages(topics=['/atv_gps']):
            gnss_points.append([msg.latitude, msg.longitude, msg.altitude])
            latitudes.append(msg.latitude)
            longitudes.append(msg.longitude)
            altitudes.append(msg.altitude)
        bag.close()
        print(gnss_points[:3])
        print(longitudes[:3])
        # calculate the mean for every dimension
        lat_arr = numpy.array(latitudes)
        lon_arr = numpy.array(longitudes)
        alt_arr = numpy.array(altitudes)
        #means
        lat_mean = numpy.mean(lat_arr, axis=0)
        lon_mean = numpy.mean(lon_arr, axis=0)
        alt_mean = numpy.mean(alt_arr, axis=0)
        # get rid of elements higher or lower than 2* the standard deviation from
        lat_sd = numpy.std(lat_arr, axis=0)
        lon_sd = numpy.std(lon_arr, axis=0)
        alt_sd = numpy.std(alt_arr, axis=0)
        
        print(len(gnss_points))
        if not "ap3_GNSS2" in bag_name:
            outlierreduced = [point for point in gnss_points if (point[0] > lat_mean - 2 * lat_sd)]
            outlierreduced = [point for point in outlierreduced if (point[0] < lat_mean + 2 * lat_sd)]
            print(outlierreduced[:3])
        else:
            # since the latitude for ap3_GNSS2 only contains same values for some reason,
            # without this all points in ap3_GNSS2 would be removed, which would cause problems for evaluation code later down the line
            outlierreduced = gnss_points
        print(len(outlierreduced))
        outlierreduced = [point for point in outlierreduced if (point[1] > lon_mean - 2 * lon_sd)]
        outlierreduced = [point for point in outlierreduced if (point[1] < lon_mean + 2 * lon_sd)]
        print(len(outlierreduced))
        outlierreduced = [point for point in outlierreduced if (point[2] > alt_mean - 2 * alt_sd)]
        outlierreduced = [point for point in outlierreduced if (point[2] < alt_mean + 2 * alt_sd)]
        print(len(outlierreduced))
   
        lat_mean_outlierreduced = numpy.mean(numpy.array([point[0] for point in outlierreduced]),axis=0)
        lon_mean_outlierreduced = numpy.mean(numpy.array([point[1] for point in outlierreduced]),axis=0)
        alt_mean_outlierreduced = numpy.mean(numpy.array([point[2] for point in outlierreduced]),axis=0)
        
        data_points_left = round((float(1)-(float(len(outlierreduced))/float(len(gnss_points)))), 2)
        print("Percentage outliers :")
        print(data_points_left)
        num_datapoints = len(gnss_points)
        return lat_mean_outlierreduced, lon_mean_outlierreduced, alt_mean_outlierreduced, lat_mean, lon_mean, alt_mean, lat_sd, lon_sd, alt_sd, num_datapoints, data_points_left


if __name__ == "__main__":
    OutlierreducedGNSS()
