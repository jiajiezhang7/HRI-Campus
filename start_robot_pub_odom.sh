#!/bin/bash
# 此shell脚本只用于启动小车底盘，其他都没有启动，适用于nav2导航研究
# 此包为了启用小车底盘发布odom->base_link的tf关系，主要体现在hunter_base_pub_odom.launch.py的参数传递中

# Change directory to the script location and run the script

# gnome-terminal --title="can2usb" -x bash -c "cd /home/agilex03/agilex_ws/src/ugv_sdk/scripts/; bash bringup_can2usb_500k.bash"
cd /home/agilex03/agilex_ws/src/ugv_sdk/scripts/ || exit

bash bringup_can2usb_500k.bash


# Change directory to the workspace and source the setup file

cd /home/agilex03/agilex_ws/ || exit

source install/setup.bash


# Launch ROS2 nodes in new gnome-terminals

gnome-terminal --title="hunter_base" -x bash -c "ros2 launch hunter_base hunter_base_pub_odom.launch.py; exec bash"


# Start PTP4L in a new gnome-terminal
gnome-terminal --title="ptp4l" -x bash -c "sudo ptp4l -m -4 -i enp2s0 -S -l 5; echo 'PTP4L finished.'; exec bash"


