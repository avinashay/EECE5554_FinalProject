
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse
import numpy as np
import pandas as pd

import rosbag
import matplotlib.pyplot as plt


def main():
    gps = np.zeros((182,2))
    bag = rosbag.Bag('/home/avinash/Downloads/lastLoop.bag', "r")
    
    i = 0
    for topic, msg, t in bag.read_messages(topics='/pub_gps_data'):
        #print(msg)
        lat, log = msg.latitude,msg.Longitude
        gps[i,:] = [degree(lat),-degree(log)]
        print(i)
        i += 1
    pd1 = pd.DataFrame(gps)
    pd1.to_csv('gps.csv')
    bag.close()

    return

def degree(x):
    degrees = int(x) // 100
    minutes = x - 100*degrees
    return degrees + minutes/60

if __name__ == '__main__':
    main()
