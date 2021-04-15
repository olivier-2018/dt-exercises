#!/bin/bash

roscore &
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/solution/devel/setup.bash
python3 solution.py &
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
roslaunch encoder_pose encoder_pose_node.launch veh:=$VEHICLE_NAME &
sleep 5
rostopic pub /$VEHICLE_NAME/activity_name std_msgs/String "data: pid_exercise"

