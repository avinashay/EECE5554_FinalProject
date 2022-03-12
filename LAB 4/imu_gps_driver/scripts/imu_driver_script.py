#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import namedtuple

import numpy as np
import rospy
import serial
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header


IMUData = namedtuple('IMUData', ['eulers',
                                 'mag',
                                 'accel',
                                 'gyro'
                                 ]
                     )
Vector3 = namedtuple('Vector3', ['x', 'y', 'z'])
Quaternion = namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

def euler_deg2rad(eulers):
    """ Helper function to convert Euler angles from degrees to radians 
    Arguments:
        eulers (np array): Euler angles in [yaw pitch roll] form in degrees
    Returns:
        eulers_rad (np array): Euler angles in [yaw pitch roll] form in radians
    """
    yaw = np.deg2rad(eulers[0])
    pitch = np.deg2rad(eulers[1])
    roll = np.deg2rad(eulers[2])
    eulers_rad = np.array([yaw, pitch, roll])
    return eulers_rad

def euler2quat_ZYX(eulers):
    """ Convert Euler angles to quaternion representation 
    
    Takes as input a set of 3 Euler angles (in radians), and calculates their
    quaternion representation using a ZYX (yaw-pitch-roll) rotation.
    
    Inputs:
        eulers (np array): Euler angles in [yaw pitch roll] form (body-Z,
            body-Y, body-X), in radians
    Returns:
        quat (namedtuple): quaternion coefficients, with fields [x y z w]
    """
    yaw = eulers[0]
    pitch = eulers[1]
    roll = eulers[2]
    
    cy = np.cos(yaw/2)
    cp = np.cos(pitch/2)
    cr = np.cos(roll/2)
    sy = np.sin(yaw/2)
    sp = np.sin(pitch/2)
    sr = np.sin(roll/2)
    
    x = np.round(sr * cp * cy - cr * sp * sy, 7)
    y = np.round(cr * sp * cy + sr * cp * sy, 7)
    z = np.round(cr * cp * sy - sr * sp * cy, 7)
    w = np.round(cr * cp * cy + sr * sp * sy, 7)
    
    quat = Quaternion(x, y, z, w)
    return quat

def init_imu(serial_port, serial_baud):
    """ Initialize IMU 
    Arguments:
        serial_port (str): name of sensor's serial port (e.g. "/dev/ttyUSB0")
        serial_baud (int): serial baud rate for sensor communications
    Returns:
        port: Python Serial port object
        imu_pub, mag_pub: ROS publisher objects
    """
    rospy.init_node('imu_data')
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.logdebug('Initializing IMU on port '
                   + serial_port
                   + ' at '
                   + str(serial_baud)
                   )
    rospy.sleep(0.2)
    
    sensor_rate = 40 # Hz
    port.write(bytes('$VNWRG,07,' + str(sensor_rate) + '*XX', 'ascii'))
    imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
    mag_pub = rospy.Publisher('magnetometer', MagneticField, queue_size=1)
    rospy.logdebug('Initialization complete. Publishing IMU data at '
                   + str(sensor_rate)
                   + ' Hz.'
                   )
    rospy.sleep(0.1)
    port.timeout = None
    return port, imu_pub, mag_pub

def decode_imu_data(line):
    """ Decode bytes data from IMU into named tuple
    Arguments:
        line (bytes): Sensor data in $VNYMR sentence format
    Returns:
        imu_data (namedtuple): contains the following fields as numpy arrays:
            eulers: Euler angles in degrees, in [yaw, pitch, roll] (ZYX) format
            mag: magnetometer x, y, and z in gauss
            accel: accelerometer x, y, and z in m/s^2
            gyro: gyroscope x, y, and z in rad/s
    """
    yaw = float(line[7:15])
    pitch = float(line[16:24])
    roll = float(line[26:33])
    mag_x = float(line[34:42])/10_000 # Convert mag data from gauss to tesla
    mag_y = float(line[43:51])/10_000
    mag_z = float(line[52:60])/10_000
    accel_x = float(line[61:68])
    accel_y = float(line[69:76])
    accel_z = float(line[77:84])
    gyro_x = float(line[85:95])
    gyro_y = float(line[96:106])
    gyro_z = float(line[107:117])
    
    imu_data = IMUData(eulers = np.array([yaw, pitch, roll]),
                       mag = Vector3(mag_x, mag_y, mag_z),
                       accel = Vector3(accel_x, accel_y, accel_z),
                       gyro = Vector3(gyro_x, gyro_y, gyro_z)
                       )
    return imu_data

def generate_imu_msg(imu_data, quats):
    """ Generate ROS Imu message (sensor_msgs/Imu.msg) """
    imu_msg = Imu(header = Header(),
                  orientation = quats,
                  orientation_covariance = np.zeros(9),
                  angular_velocity = imu_data.gyro,
                  angular_velocity_covariance = np.zeros(9),
                  linear_acceleration = imu_data.accel,
                  linear_acceleration_covariance = np.zeros(9),
                  )
    return imu_msg

def generate_magnetometer_msg(imu_data):
    """ Generate ROS MagneticField message (sensor_msgs/MagneticField.msg) """
    mag_msg = MagneticField(header = Header(),
                            magnetic_field = imu_data.mag,
                            magnetic_field_covariance = np.zeros(9)
                            )
    return mag_msg

if __name__ == '__main__':
    port, imu_pub, mag_pub = init_imu('/dev/ttyUSB0', 115200)
    try:
        while not rospy.is_shutdown():
            # Wait for valid $VNYMR message before proceeding
            is_vnymr = False
            while not is_vnymr:
                line = port.readline()
                if line[0:6] == b'$VNYMR':
                    is_vnymr = True
            # Decode data and convert Euler angles to quaternion representation
            imu_data = decode_imu_data(line)
            eulers_rad = euler_deg2rad(imu_data.eulers)
            quats = euler2quat_ZYX(eulers_rad)
            
            # Write data to ROS messages and publish
            imu_msg = generate_imu_msg(imu_data, quats)
            mag_msg = generate_magnetometer_msg(imu_data)
            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)

    except rospy.ROSInterruptException:
        port.close()
    except serial.serialutil.SerialException:
        rospy.loginfo('Shutting down imu_data node')
