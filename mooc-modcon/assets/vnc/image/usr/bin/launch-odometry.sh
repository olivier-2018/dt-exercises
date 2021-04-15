#!/bin/bash
rostopic pub /activity_name std_msgs/String "data: odometry" & dt-launcher-joystick & rviz -d /opt/ros/noetic/share/rviz/odometry.rviz