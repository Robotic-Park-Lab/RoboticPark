#! /bin/bash

echo -e "Robotic Park install\n"
echo -e "Checking and installing dependencies to build ..."

cd ../..
source install/setup.bash

cd src

echo -e "\nCrazyflie package:"

var0="$(ros2 pkg prefix uned_crazyflie_config)"

if [ -z "$var0" ]; then
    echo -e "\tInstalling ..."
    git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg.git
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
    git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_swarm_ros_pkg.git
else
    echo -e "\tInstalled."
fi

echo -e "\nKhepera IV package:"

var0="$(ros2 pkg prefix uned_kheperaiv_config)"

if [ -z "$var0" ]; then
    echo -e "\tInstalling ..."
    git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg.git
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

