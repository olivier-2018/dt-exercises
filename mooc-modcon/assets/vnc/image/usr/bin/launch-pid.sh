#!/bin/bash
rviz -d /opt/ros/noetic/share/rviz/odometry.rviz &
rostopic pub /activity_name std_msgs/String "data: pid"
#rostopic pub /$VEHICLE_NAME/joy_mapper_node/car_cmd duckietown_msgs/Twist2DStamped "{header: auto,  v: 0, omega: 0}"