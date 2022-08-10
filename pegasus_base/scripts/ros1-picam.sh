#!/bin/bash
/sbin/iw dev wlan0 set power_save off
export ROS_MASTER_URI=http://10.0.0.15:11311 
export ROS_HOSTNAME=10.0.0.15 
export ROS_IP=10.0.0.15
source /opt/ros/noetic/setup.bash
source /home/trentxavier/ws_ros1/devel/setup.bash

roslaunch pegasus_base pegasus_apriltag.launch
echo "Completed..."

