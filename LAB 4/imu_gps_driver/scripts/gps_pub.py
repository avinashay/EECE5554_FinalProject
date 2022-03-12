#
# gps_pub.py

import rospy
import serial
import utm
import numpy as np
from std_msgs.msg import String, Header
from imu_gps_driver.msg import gps_data_format as msg_format


def gps_pub():
    pub = rospy.Publisher('gps_pub', msg_format, queue_size=20)
    rospy.init_node('gps_pub_node', anonymous=True)
    rate = rospy.Rate(10)

    fliter_head = 'GPGGA'

    serial_port = rospy.get_param('~port', '/dev/ttyUSB1')
    serial_baud = rospy.get_param('~baudrate', 4800)

    port = serial.Serial(serial_port, serial_baud, timeout=3)
    while not rospy.is_shutdown():
        msg_raw = port.readline().decode('latin-1')
        msg_filter = logs_filter(msg_raw, fliter_head)  # gpgga latitude latDir longitude lonDir altitude
        #print(msg_filter)

        if msg_filter.size >= 2:
            msg_utm = np.array(utm.from_latlon(np.double(msg_filter[1]), np.double(msg_filter[3])))  # utm_e utm_n zone letter
            #print(msg_utm)
            #
            msg = msg_format()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.latitude = np.double(msg_filter[1])
            msg.longitude = np.double(msg_filter[3])
            msg.altitude = np.double(msg_filter[5])
            msg.utm_easting = np.double(msg_utm[0])
            msg.utm_northing = np.double(msg_utm[1])
            msg.zone = int(msg_utm[2])
            msg.letter = str(msg_utm[3])

            pub.publish(msg)
            rate.sleep()


# pick GPGGA data   # GPGGA latitude latDir longitude lonDir altitude
def logs_filter(msg, filter_head):
    msg_out = np.asarray(msg.split(','))

    if filter_head in msg_out[0]:
        if msg_out[2] and msg_out[4] and msg_out[9]:
            if 'S' in msg_out[3]:
                msg_out[2] = -deg2dci(np.double(msg_out[2]))
            else:
                msg_out[2] = deg2dci(np.double(msg_out[2]))
            if 'W' in msg_out[5]:
                msg_out[4] = -deg2dci(np.double(msg_out[4]))
            else:
                msg_out[4] = deg2dci(np.double(msg_out[4]))

            return msg_out.take([0, 2, 3, 4, 5, 9])
        else:
            return np.array(1)
    else:
        return np.array(1)


# def

def deg2dci(deg_num):
    degree = int(deg_num) // 100
    minute = np.double(deg_num) - 100 * degree
    return np.double(degree + (minute / 60))


if __name__ == '__main__':
    try:
        gps_pub()
    except rospy.ROSInterruptException:
        pass
