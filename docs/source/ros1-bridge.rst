sudo apt-get update
sudo apt-get install -y ros-galactic-ros1-bridge

source /opt/ros/noetic/setup.bash

source /opt/ros/galactic/setup.bash

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics