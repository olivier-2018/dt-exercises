#!/bin/bash

source /environment.sh
source /code/exercise_ws/devel/setup.bash

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app

python3 /code/solution.py &> /dev/null &
dt-exec roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
dt-exec roslaunch controller_pkg controller_node.launch veh:=$VEHICLE_NAME
# roslaunch interaction_pkg interaction_node.launch veh:=$VEHICLE_NAME
#python3 /code/exercise_ws/src/interaction_pkg/src/interaction_node.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
