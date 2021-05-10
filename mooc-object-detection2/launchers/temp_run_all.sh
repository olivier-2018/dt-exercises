#!/bin/bash
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/exercise_ws/devel/setup.bash
python3 /code/solution.py &
echo "Launching car_interface"
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
echo "Launching duckietown_demos"
roslaunch --wait duckietown_demos lane_following.launch &
sleep 5
echo "Switching to LANE_FOLLOWING"
rostopic pub /$VEHICLE_NAME/fsm_node/mode duckietown_msgs/FSMState '{header: {}, state: "LANE_FOLLOWING"}'
