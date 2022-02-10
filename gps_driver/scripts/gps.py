#!/usr/bin/env python3
import serial
import rospy 
import utm
from std_msgs.msg import Header
from gps_driver.msg import GPS as gps_msg
import numpy as np

def degree(x):
    degrees = int(x) // 100
    minutes = x - 100*degrees
    return degrees + minutes/60

def gps():
    rospy.init_node('gps_node')
    pub = rospy.Publisher("/utm_data",gps_msg,queue_size=10)
    serial_port = rospy.get_param('~port','/dev/ttyUSB1')
    serial_baud = rospy.get_param('~baudrate',4800)
    gps = serial.Serial(port=serial_port,baudrate=serial_baud, timeout=5.)
    rate = rospy.Rate(10)
    msg = gps_msg()
    while not rospy.is_shutdown():
            data = str(gps.readline())
            data_split = data.split(",")
            #print(data_split)
            #print("waiting for the data")
            if data_split[0] == "b'$GPGGA":
                if data_split[2] != '' and data_split[4] != '' :
                    latitude = degree(float(data_split[2]))
                    longitude = -degree(float(data_split[4]))
                    altitude = float(data_split[9])
                    position_utm = np.array(utm.from_latlon(latitude,longitude))
                    print(position_utm)
                    msg.header = Header()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = 'GPS DATA'
                    msg.lat = latitude
                    msg.log = longitude
                    msg.alt = altitude
                    msg.utm_easting = float(position_utm[0])
                    msg.utm_northing = float(position_utm[1])
                    msg.zone_num = int(position_utm[2])
                    msg.zone_letter = position_utm[3]
                    pub.publish(msg)
                    rate.sleep()


if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass