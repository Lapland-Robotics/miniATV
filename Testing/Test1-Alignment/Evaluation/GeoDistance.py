# -*- coding: utf-8 -*-
"""
by C. Stubenvoll

"""
import csv
from geopy import distance
import numpy as np

class GeoDistance():
    def __init__(self):
        #Load csv files for the reference points and the gnss measurements from test1
        print("before")
        test1_averages = 'outlierreduced_gnss_test1.csv'
        test1_ref_ponts = 'reference_points_both_xyz_latlongalt.csv'
        # D:\Dokumente\Dokumente\ISY\Masterarbeit\Test1_Evaluation
        print("after")
        
        with open(test1_ref_ponts, mode='r') as csv_file:
            reader= csv.DictReader(csv_file)
            reference = list(csv.reader(csv_file, delimiter=";"))
            csv_file.close()
        #print(reference)
        with open(test1_averages, mode='r') as csv_file:
            reader= csv.DictReader(csv_file)
            averages = list(csv.reader(csv_file, delimiter=","))
            csv_file.close()
            
        dists = self.distances(reference, averages)
        
        
         
    def distances(self, reference, averages):



        print (reference[0])
        rounded_dist_rows = []
        dist_rows = []
        dist_header = ['Measurement Name', 'On Reference Point','With GNSS', 'outlierreduced distance [m]', 'distance (non corrected) [m]', 'difference corrected uncorrected', 'percentage outliers']
        
        
        # Distances btw each reference point and the GNSS measurements belonging to that
        for ref in reference[1:]:
            print(ref[0])
            #ref_csv_rows.append[ref[0], ref_carla_location, ref_carla_location.x, ref_carla_location.y, ref_carla_location.z]
            #ref_dict[ref[0]] = {ref_carla_location}
            for avg in averages[1:]:
                if ref[0] in avg[0]:
                    
                    avg_key = str(avg[0])[:-24] # get rid of the numbers in the name
                    print(avg_key)
                    gnss_key = avg_key[-5:] # get only the name of GNSS antenna used
                    print(gnss_key)
                    # get lat lon alt of outlierreduced mean for this measurement
                    avg_lat = avg[1]
                    avg_lon = avg[2]
                    avg_alt = avg[3]
                    # get lat lon alt of not outlierreduced mean for this measurement for comparison
                    wo_lat = avg[4]
                    wo_lon = avg[5]
                    wo_alt = avg[6]
                    
                    dist_outlierreduced = self.distance_3d(float(ref[4]), float(ref[5]), float(ref[3]), float(avg_lat), float(avg_lon), float(avg_alt))
                    dist_wo_correction = self.distance_3d(float(ref[4]), float(ref[5]), float(ref[3]), float(wo_lat), float(wo_lon), float(wo_alt))
                    diff = dist_wo_correction - dist_outlierreduced
                    rounded_dist_outlierreduced = round(dist_outlierreduced, 3)
                    rounded_dist_wo_correction = round(dist_wo_correction, 3)
                    rounded_diff = round(diff, 3)
                    rounded_row = [avg[0], ref[0], gnss_key, rounded_dist_outlierreduced, rounded_dist_wo_correction, rounded_diff, avg[11]]
                    row = [avg[0], ref[0], gnss_key, dist_outlierreduced, dist_wo_correction, diff, avg[11]]
                    rounded_dist_rows.append(rounded_row)
                    dist_rows.append(row)
                    
            
        
        test1_rounded_coord_dists = 'test1_rounded_distances_coordinate_to_reference.csv'
        # write the carla positions to a csv file (in case of evaluation outside of carla)
        with open(test1_rounded_coord_dists, 'w') as rdistfile:
            rdist_writer = csv.writer(rdistfile) # creating a csv writer object
            rdist_writer.writerow(dist_header)
            rdist_writer.writerows(rounded_dist_rows)
            rdistfile.close()
            


        test1_coord_dists = 'test1_distances_coordinate_to_reference.csv'
        # write the carla positions to a csv file (in case of evaluation outside of carla)
        with open(test1_coord_dists, 'w') as distfile:
            dist_writer = csv.writer(distfile) # creating a csv writer object
            dist_writer.writerow(dist_header)
            dist_writer.writerows(dist_rows)
            distfile.close()
        return dist_rows
    
    
    def distance_3d(self, lat_a, lon_a, alt_a, lat_b, lon_b, alt_b):
        # lat, lon, alt [m]
        pt_a = [lat_a, lon_a, alt_a]
        pt_b = [lat_b, lon_b, alt_b]
        # 2D geodesic distance [m]
        dist_2d= distance.distance(pt_a[:2], pt_b[:2]).m
        # 3D euclidean distance
        dist_3d = np.sqrt(dist_2d**2 + (pt_b[2] - pt_a[2])**2)
        print(dist_3d)
        return dist_3d
        
        
if __name__ == "__main__":
    GeoDistance()