#!/bin/bash

source /environment.sh
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend

git clone https://github.com/Velythyl/dataclasses-spoof.git dataclasses && cd dataclasses &&  pip3 install . && cd ..
pip3 install git+https://github.com/velythyl/lib-dt-mooc-2021 # TODO change this to duckietown once we're merged...
#pip3 install pycuda
pip3 install cython

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app

dt-exec roslaunch --wait agent agent_node.launch
dt-exec roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME
# TODO LIAM is this right? Refactored the old launcher to this (found under the separator line ---)
dt-exec roslaunch --wait duckietown_demos lane_following_pedestrians.launch


# ----------------------------------------------------------------------------

# wait for app to end
dt-launchfile-join


#!/bin/bash
#source /environment.sh
#source /opt/ros/noetic/setup.bash
#source /code/catkin_ws/devel/setup.bash
#source /code/exercise_ws/devel/setup.bash
#python3 /code/solution.py &
#roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
#roslaunch --wait duckietown_demos lane_following_pedestrians.launch
