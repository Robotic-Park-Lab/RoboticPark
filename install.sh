#! /bin/bash

echo -e "Robotic Park install\n"
echo -e "Checking and installing dependencies to build ..."

sudo apt install ros-humble-webots-ros2

cd ../..
source install/setup.bash

cd src

echo -e "\nWebots:"
var0="$(apt list --installed | grep webots/now)"

if [ -z "$var0" ]; then
    sudo mkdir -p /etc/apt/keyrings
    cd /etc/apt/keyrings
    sudo wget -q https://cyberbotics.com/Cyberbotics.asc
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
    sudo apt update
    sudo apt install webots
else
    echo -e "\tInstalled."
fi

echo -e "\nCrazyflie package:"

var0="$(ros2 pkg prefix uned_crazyflie_config)"

if [ -z "$var0" ]; then
    echo -e "\tInstalling ..."
    git clone -b benchmark https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg.git
else
    echo -e "\tInstalled."
fi

echo -e "\nVicon package:"

var0="$(ros2 pkg prefix vicon_receiver)"

if [ -z "$var0" ]; then
    echo -e "\tInstalling ..."
    git clone -b main https://github.com/Robotic-Park-Lab/ros2-vicon-receiver.git
else
    echo -e "\tInstalled."
fi

echo -e "\nSwarm package:"

var0="$(ros2 pkg prefix uned_swarm_config)"

if [ -z "$var0" ]; then
    echo -e "\tInstalling ..."
    git clone -b benchmark https://github.com/Robotic-Park-Lab/uned_swarm_ros_pkg.git
else
    echo -e "\tInstalled."
fi

echo -e "\nKhepera IV package:"

var0="$(ros2 pkg prefix uned_kheperaiv_config)"

if [ -z "$var0" ]; then
    echo -e "\tInstalling ..."
    git clone -b benchmark https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg.git
else
    echo -e "\tInstalled."
fi

cd 
echo -e "\nCrazyflie-lib-python:"
dirname='Code'

if [ -d "$dirname" ]; then
    cd Code
    dirname='crazyflie-lib-python'
    if [ -d "$dirname" ]; then
        echo -e "\tInstalled."
    else
        git clone https://github.com/Robotic-Park-Lab/crazyflie-lib-python.git
        cd crazyflie-lib-python
        pip install -e .
        cd
    fi
else
    mkdir Code
    cd Code
    git clone https://github.com/Robotic-Park-Lab/crazyflie-lib-python.git
    cd crazyflie-lib-python
    pip install -e .
    cd
fi

cd ~/roboticpark_ws/
rosdep update
rosdep install --from-paths src -y --ignore-src
# colcon build --symlink-install
source install/setup.bash
cd ~/roboticpark_ws/src/RoboticPark

