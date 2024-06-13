#!/bin/bash

cd /docker-scripts/launch-scripts/

echo "launch micrortps #1"
if ! screen -list | grep -q "micrortps_1"; then
    screen -dmS micrortps_1 ./launch_micrortps_1.sh
    echo "micrortps_1 started"
else
    echo "micrortps_1 already running"
fi

echo "launch micrortps #2"
if ! screen -list | grep -q "micrortps_2"; then
    screen -dmS micrortps_2 ./launch_micrortps_2.sh
    echo "micrortps_2 started"
else
    echo "micrortps_2 already running"
fi

echo "launch micrortps #3"
if ! screen -list | grep -q "micrortps_3"; then
    screen -dmS micrortps_3 ./launch_micrortps_3.sh
    echo "micrortps_3 started"
else
    echo "micrortps_3 already running"
fi

echo "launch px4_ros_api"
if ! screen -list | grep -q "px4_ros_api"; then
    screen -dmS px4_ros_api ./launch_px4_ros_api.sh
    echo "px4_ros_api started"
else
    echo "px4_ros_api already running"
fi