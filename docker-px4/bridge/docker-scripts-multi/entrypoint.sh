#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /root/px4_ros_com_ros2/install/setup.bash

# Welcome information
echo "---------------------"
echo "PX4 ROS2 Docker Image"
echo "---------------------"
echo 'OS distribution info:'
lsb_release -cdr
echo "---------------------"
echo 'Computer architecture:'
dpkg-architecture -q DEB_BUILD_ARCH
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'DDS middleware: ' $RMW_IMPLEMENTATION 
# echo 'Domain ID: ' $ROS_DOMAIN_ID 
echo "---"  
echo 'Available px4 packages:'
ros2 pkg list | grep px4
echo "---------------------"    
exec "$@"

/docker-scripts/startup_script.sh

/bin/bash
