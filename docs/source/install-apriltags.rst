<<<<<<< HEAD
cd /(ws_name)/src
git clone https://github.com/Adlink-ROS/apriltag_ros.git
git clone https://github.com/AprilRobotics/apriltag.git

Open up the tag_realsense.launch.py and edit the following line to match the topics of image and camera info: 

image_topic_ = LaunchConfiguration('image_topic', default="image_rect")
camera_name = LaunchConfiguration('camera_name', default="/camera_pub")

cd /(ws_name)
colcon build

Run camera publisher:
cd /(ws_name)/pegasus-mini-ros2-dev/pegasus_base/pegasus_base
python3 pi_cam_node.py

launch apriltag package:
ros2 launch apriltag_ros tag_realsense.launch.py
=======
sudo apt-get install ros-foxy-cv-bridge
sudo apt install ros-foxy-image-geometry
>>>>>>> 740ebcc698548c41475114ca22058bed5a8bff0c
