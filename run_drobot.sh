#!/bin/bash
pkill -9 gz 2>/dev/null
pkill -9 ruby 2>/dev/null
sleep 1

cd ~/Desktop/PX4-Autopilot/build/px4_sitl_default/rootfs
source gz_env.sh
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/kj/Desktop/drobot/ros2_ws/src
cd ~/Desktop/PX4-Autopilot
PX4_SYS_AUTOSTART=4601 PX4_SIM_MODEL=drobot ./build/px4_sitl_default/bin/px4
