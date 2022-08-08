Add ROS2 apt repo to your system:
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

Add repo to source list:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

Install ROS 2 Foxy: 
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop

Create Workspace:

sudo apt install python3-colcon-common-extensions #Install Colon
source /opt/ros/foxy/setup.bash

mkdir -p ~/(ws_name)/src
cd ~/(ws_name)/src


git clone https://github.com/protogarden/pegasus-mini-ros2-dev.git
git clone https://github.com/Slamtec/sllidar_ros2.git
sudo apt install python3-pip
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
sudo apt install python3-pip
pip3 install smbus
pip3 install pyserial
sudo apt install python3-rosdep2
rosdep update
rosdep install -i --from-path src --rosdistro foxy -y
sudo apt-get install ros-foxy-tf2-tools ros-foxy-tf-transformations
sudo pip3 install transforms3d
sudo apt install python3-rostopic



Build Packages:
colcon build
colcon build --symlink-install 

Add the following to .bashrc:
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source ~/(ws_space)/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/foxy/" >> ~/.bashrc

source ./install/setup.bash
source ~/.bashrc
sudo adduser 'trentpc-ros2' dialout
///
ros2 launch teleop_twist_joy  teleop-launch.py joy_config:='xbox'
ros2 launch pegasus_base pegasus_base.launch.py
ros2 launch pegasus_navigation pegasus_nav.launch.py 
ros2 run joy joy_node
///

ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /home/trentxavier/ws_bot/src/pegasus-mini-ros2-dev/pegasus_navigation/map/map.yaml}"
sudo /sbin/iw dev wlan0 set power_save off
sudo adduser 'trentpc-ros2' dialout