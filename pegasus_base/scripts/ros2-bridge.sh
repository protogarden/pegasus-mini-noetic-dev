#!/bin/bash

export ROS_DOMAIN_ID=1
source /opt/ros/galactic/setup.bash
source /opt/ros/noetic/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics
