#!/bin/bash

micrortps_agent -t UDP -r 2020 -s 2019 -n ${ROS_NAMESPACE} --ros-args -r __ns:=/${ROS_NAMESPACE}