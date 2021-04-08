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

python3 /code/solution.py &
dt-exec roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &


dt-exec roslaunch encoder_pose encoder_pose_node.launch veh:=$VEHICLE_NAME


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
