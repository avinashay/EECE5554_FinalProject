
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2
import time
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():

    bag = rosbag.Bag('/home/avinash/Downloads/big.bag', "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics='/camera_array/cam1/image_raw'):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join('./bagtoimg/big/image_1', '0'*(6-len(str(count))) + str(count) + ".png"), cv_img)
        print("Wrote image",count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()