#!/usr/bin/env python3
import sys
import os
import csv
import rosbag
import bagpy
from bagpy import bagreader
import rospy
import numpy as np
import pandas as pd

##################
# DESCRIPTION:
# Creates CSV files of the robot joint states from a rosbag (for visualization with e.g. pybullet)
# 
# USAGE EXAMPLE:
# rosrun your_package get_jstate_csvs.py /root/catkin_ws/bagfiles your_bagfile.bag
# ##################

x = 0
bag = bagreader("/home/avinash/catkin_ws/src/gps_rtk/data/lab2_open_stat.bag")


print("Writing to CSV")



csvfiles = []

for t in bag.topics:
  data = bag.message_by_topic(t)
  csvfiles.append(data)

print(csvfiles)

