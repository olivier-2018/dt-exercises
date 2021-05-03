#!/bin/bash
sed -i "s/agent/${VEHICLE_NAME}/g" /opt/ros/noetic/share/rviz/viscontrol.rviz
rviz -d /opt/ros/noetic/share/rviz/viscontrol.rviz &
rostopic pub /$VEHICLE_NAME/activity_name std_msgs/String "data: vls_exercise"
