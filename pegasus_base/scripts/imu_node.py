#!/usr/bin/env python

import rospy
import serial
import string
import math
import sys
import tf
import tf2_ros
import geometry_msgs.msg
import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0
debug = 0



rospy.init_node("imu_node")

imuMsg = Imu()

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.


# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
0.02 , 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# read basic information
port = rospy.get_param('~port', '/dev/ttyUSB1')
gx_offset = rospy.get_param('~gx_offset', 0.0)
gy_offset = rospy.get_param('~gy_offset', 0.0)
gz_offset = rospy.get_param('~gz_offset', 0.0)
topic = rospy.get_param('~topic', 'imu')
vth_thresh = rospy.get_param('~vth_thresh', 0.02)
frame_id = rospy.get_param('~frame_id', 'base_link')
imu_node_freq =  rospy.get_param('~imu_node_frequency', 100)


pub = rospy.Publisher(topic, Imu, queue_size=1)
br = tf.TransformBroadcaster()
last_time = rospy.Time.now()

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
rate = rospy.Rate(imu_node_freq)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
    #ser = serial.Serial(port=port, baudrate=57600, timeout=1, rtscts=True, dsrdtr=True) # For compatibility with some virtual serial ports (e.g. created by socat) in Python 2.7
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

roll=0
pitch=0
yaw=0
seq=0

rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = bytearray(ser.readline()).decode("utf-8")
    #print(line)
   
rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

errcount = 0
while not rospy.is_shutdown():
    if (errcount > 10):
        break
    line = bytearray(ser.readline()).decode("utf-8")


    ser.flush()
    if ((line.find("YPRAxAyAzGxGyGz=") == "") or (line.find("\r\n") == "")): 
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    else:
        errcount = 0
    line = line.replace("YPRAxAyAzGxGyGz=","")   # Delete "YPRAxAyAzGxGyGz="
    #f.write(line)                     # Write to the output log file
    line = line.replace("\r\n","")   # Delete "\r\n"
    words = line.split(",")    # Fields split
    if len(words) != 9:
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    else:
        errcount = 0
        yaw_deg = float(words[0])
        yaw_deg = yaw_deg + imu_yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        


        pitch_deg = float(words[1])
        roll_deg = float(words[2])
        #print(yaw_deg)
  
        yaw = yaw_deg*degrees2rad
        pitch = pitch_deg*degrees2rad
        roll = roll_deg*degrees2rad

        imuMsg.linear_acceleration.x = float(words[3])
        imuMsg.linear_acceleration.y = float(words[4])
        imuMsg.linear_acceleration.z = -float(words[5])

        imuMsg.angular_velocity.x = float(words[6])
        imuMsg.angular_velocity.y = float(words[7])
        imuMsg.angular_velocity.z = float(words[8])

    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = frame_id
    imuMsg.header.seq = seq
    seq = seq + 1
    current_time = rospy.Time.now()
    d_time = (current_time - last_time).to_sec()
    last_time = current_time
    if d_time > 0.015 or (imuMsg.angular_velocity.z < vth_thresh and imuMsg.angular_velocity.z > -vth_thresh):
        
        
        imuMsg.orientation_covariance = [
            99999 , 0 , 0,
            0, 99999, 0,
            0, 0, 99999
            ]


    else: 
   
        imuMsg.orientation_covariance = [
            0.002 , 0 , 0,
            0, 0.002, 0,
            0, 0, 0.002
            ]


    pub.publish(imuMsg)
    rate.sleep()
    if debug == 1:
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)
   
ser.close
#f.close
