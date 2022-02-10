#!/usr/bin/env python3
import sys
import os
import csv
import rosbag
import rospy
import numpy as np

##################
# DESCRIPTION:
# Creates CSV files of the robot joint states from a rosbag (for visualization with e.g. pybullet)
# 
# USAGE EXAMPLE:
# rosrun your_package get_jstate_csvs.py /root/catkin_ws/bagfiles your_bagfile.bag
# ##################

x = 0
bag = rosbag.Bag("/home/avinash/catkin_ws/src/gps_driver/data/redeye.bag")


print("Writing to CSV")

with open('/home/avinash/catkin_ws/src/gps_driver/data/redeye.csv', mode='w') as data_file:
  data_writer = csv.writer(data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  data_writer.writerow(['sequence','time','latitude', 'logitude','altitude','utm_easting','utm_northing', 'zone_num', 'zone_letter'])
  for topic, msg, t in bag.read_messages(topics=['/utm_data']):
    # Only write to CSV if the message is for our robot
    data_writer.writerow([msg.header.seq,msg.header.stamp.secs,msg.lat,msg.log,msg.alt,msg.utm_easting,msg.utm_northing,msg.zone_num,msg.zone_letter])
    print(x,msg)
    x = x+1
print("Finished creating csv file!")
bag.close()