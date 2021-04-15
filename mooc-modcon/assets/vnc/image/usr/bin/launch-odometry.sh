#!/bin/bash
rostopic pub /$VEHICLE_NAME/activity_name std_msgs/String "data: odometry" &
dt-launcher-joystick & 
rviz -d /opt/ros/noetic/share/rviz/odometry.rviz