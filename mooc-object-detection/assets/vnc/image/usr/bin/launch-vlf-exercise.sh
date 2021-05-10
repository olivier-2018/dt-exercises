#!/bin/bash
sed -i "s/agent/${VEHICLE_NAME}/g" /opt/ros/noetic/share/rviz/viscontrol.rviz
rviz -d /opt/ros/noetic/share/rviz/viscontrol.rviz &
python3 /root/Documents/VLF-control-tool.py &
dt-launcher-joystick &
rostopic pub --latch "/${VEHICLE_NAME}/vlf_node/action" std_msgs/String "data: init"