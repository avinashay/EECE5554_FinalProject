#!/usr/bin/env python3
import serial
import rospy 
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from squaternion import Quaternion




def imu():
    rospy.init_node('imu_node')
    pub_imu = rospy.Publisher('/imu',Imu,queue_size=10)
    pub_mag = rospy.Publisher('/magnetometer',MagneticField,queue_size=10)
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',115200)
    imu_data = serial.Serial(port=serial_port,baudrate=serial_baud, timeout=None)
    imu_data.write(b'$VNWRG,07,40*XX')
    msg_imu = Imu()
    msg_mag = MagneticField()
    while not rospy.is_shutdown():
        line = imu_data.readline().decode('utf-8')
        line = line.split(",")
        j = 0
        for i in line:
            i = str(i)
            if '\x00' in i :
                i = i.replace('\x00','')
                line[j] = i
            j = j+1
        if line[0] == '$VNYMR' and len(line) == 13: 
            print(line)
            yaw = float(line[1])
            pitch = float(line[2])
            roll = float(line[3])
            mag_x = float(line[4]) 
            mag_y = float(line[5])
            mag_z = float(line[6])
            accel_x = float(line[7])
            accel_y = float(line[8])
            accel_z = float(line[9])
            gyro_x = float(line[10])
            gyro_y = float(line[11])
            if '*' in line[12]:
                gz = line[12].split('*')
                gyro_z = float(gz[0])
            else :
                gyro_z = float(line[12])
            q = Quaternion.from_euler(roll,pitch,yaw, degrees=True)

            # publishing IMU data to /imu topic 
            msg_imu.header = Header()
            msg_imu.header.stamp = rospy.Time.now()
            msg_imu.header.frame_id = 'IMU_DATA'
            msg_imu.orientation.x = q.vector[0]
            msg_imu.orientation.y = q.vector[1]
            msg_imu.orientation.z = q.vector[2]
            msg_imu.orientation.w = q.scalar
            msg_imu.angular_velocity.x = gyro_x
            msg_imu.angular_velocity.y = gyro_y
            msg_imu.angular_velocity.z = gyro_z
            msg_imu.linear_acceleration.x = accel_x
            msg_imu.linear_acceleration.y = accel_y
            msg_imu.linear_acceleration.z = accel_z
            pub_imu.publish(msg_imu)

            # publishing magnetometer data to /magnetometer topic
            msg_mag.header = Header()
            msg_mag.header.stamp = rospy.Time.now()
            msg_mag.header.frame_id = 'Magnetometer_Data'
            msg_mag.magnetic_field.x = mag_x
            msg_mag.magnetic_field.y = mag_y
            msg_mag.magnetic_field.z = mag_z
            pub_mag.publish(msg_mag)

       
    

if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        pass
