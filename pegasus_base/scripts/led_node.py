#!/usr/bin/env python
import rospy
import sys
import serial
from std_msgs.msg import Int32

def callback(data):
    cmd = data.data
    
    if cmd != None:
        ser.write(bytes(cmd) + '\r\n')
        #ser.write(bytes(cmd))
        


rospy.init_node("led_node")
port = rospy.get_param('~port', '/dev/ttyUSB0')
rospy.Subscriber("led_cmd", Int32, callback)
rate = rospy.Rate(30)



try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
    #ser = serial.Serial(port=port, baudrate=57600, timeout=1, rtscts=True, dsrdtr=True) # For compatibility with some virtual serial ports (e.g. created by socat) in Python 2.7
except serial.serialutil.SerialException:
    rospy.logerr("LED Processor not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

while not rospy.is_shutdown():
  

    rospy.spin()

ser.close



