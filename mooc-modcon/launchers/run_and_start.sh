#!/bin/bash

roscore &
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/submission_ws/devel/setup.bash
python3 solution.py &
roslaunch --wait encoder_pose encoder_pose_node.launch veh:=$VEHICLE_NAME &
sleep 5
rostopic pub /activity_name std_msgs/String "data: pid_exercise"

