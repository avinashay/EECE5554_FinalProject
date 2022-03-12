import sys
import os
import csv
import rosbag
import rospy

file_bag = 'st1.bag'
dir_bag = '/home/tse/catkin_ws/src/gps_driver/data/'
bag = rosbag.Bag(dir_bag+file_bag)
#extension = ''
print('read')

result_dir = dir_bag + file_bag[:-4] + '_result'
#if not os.path.exists(result_dir):
    # os.makedirs(result_dir)
print('write')

with open(result_dir + '.csv', mode = 'w') as file_csv:
    csv_writer = csv.writer(file_csv, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    csv_writer.writerow(['time', 'header', 'lat', 'lon', 'alt', 'u_e', 'u_n', 'zone', 'letter'])

    for topic, msg, t in bag.read_messages(topics=['/gps_pub']):
        csv_writer.writerow([t, msg.header, msg.latitude, msg.longitude, msg.altitude, msg.utm_easting, msg.utm_northing, msg.zone, msg.letter])

print('finish')
bag.close
