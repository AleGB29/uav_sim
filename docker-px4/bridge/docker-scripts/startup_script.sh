#!/bin/bash

cd /docker-scripts/launch-scripts/

echo "launch micrortps"
if ! screen -list | grep -q "micrortps"; then
    screen -dmS micrortps ./launch_micrortps.sh
    echo "micrortps started"
else
    echo "micrortps already running"
fi

echo "launch px4_ros_api"
if ! screen -list | grep -q "px4_ros_api"; then
    screen -dmS px4_ros_api ./launch_px4_ros_api.sh
    echo "px4_ros_api started"
else
    echo "px4_ros_api already running"
fi