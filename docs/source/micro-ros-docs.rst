MAKE MICRO_ROS PICO FIRMWARE:

On seprate ubuntu machine running 20.04 whichyou have SSH into with VSCODE, Run the following: 
sudo apt install build-essential cmake gcc-arm-none-eabi libnewlib-arm-none-eabi doxygen git python3

mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws/src
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
git clone https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git
cd ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk
mkdir .vscode
touch .vscode/settings.json

Add the following to settings.json:
{
    "cmake.configureEnvironment": {
        "PICO_SDK_PATH": "/home/$USER/micro_ros_ws/src/pico-sdk",
    },
}

The following will bring up a new VS-Code window:
code .
#In order to bring this up in th future:
cd ~/micro_ros_ws/src/micro_ros_raspberrypi_pico_sdk
code .

Once VS-Code bring up a new window go to extesnions and install C++ extension and CMake tools for VSCode onto host machine.
Open the palette (ctrl+shift+p) and search for CMake: Scan for Kits and then CMake: Select a Kit and make sure to select the compiler we’ve installed above, that is GCC for arm-non-eabi
CMake: Build to build, and uf2 file to pico

INSTALL MICRO-ROS IN HOST COMPUTER:

source /opt/ros/foxy/setup.bash
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
sudo apt-get install python3-pip
# Build micro-ROS tools and source them
colcon build
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
# Download micro-ROS agent packages
ros2 run micro_ros_setup create_agent_ws.sh
# Build step
ros2 run micro_ros_setup build_agent.sh
source ~/.bashrc

In order to run:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 ROS_DOMAIN_ID=0


