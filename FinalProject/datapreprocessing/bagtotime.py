#!/usr/bin/env python

import os
import argparse

import time
import rosbag


def main():

    bag = rosbag.Bag('/home/avinash/Downloads/big.bag', "r")
    count = 0
    f1=open("./bagtoimg/big/times.txt","a")
    for topic, msg, t in bag.read_messages(topics='/camera_array/cam0/image_raw'):
        print((t.to_nsec()-1580572316137308877)/(10**9))
        f1.write(f"{(t.to_nsec()-1580572316137308877)/(10**9)} \n")
    f1.close()

    bag.close()

    return

if __name__ == '__main__':
    main()
