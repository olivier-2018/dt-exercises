>> What are states? <<

[ ] Physical territories delimited by borders, where the robot can operate.
[x] Mathematical variables that contain relevant information for accomplishing the task.
[ ] Position, speed, and acceleration of the robot.
[ ] Position, speed, and acceleration of the robot and every other object in its perception field.

[explanation]
States are mathematical variables that enable any robot to create a meaningful representation of the environment and its relation to it.
[explanation]

>> Why are states useful? <<

[ ] They allow decentralizing computation to regional clusters, freeing up resources for the robot.
[ ] They enable the communication of information between different modules of the agent.
[ ] They are computationally efficient.
[x] They enable the robot to create an estimate, or belief, of the world around it and its relation to it.

[explanation]
States synthetically represent what is important for the robot to accomplish its task.
[explanation]

>> What is the main difference between observations and states? <<

[x] Observations contain data, states contain actionable information synthesized from the data.
[ ] Observations are subjective, state estimates are objective.
[ ] Observations are noisy, states are not.
[x] Observations are actual measurements coming from sensors, states are extracted from these measurements.

[explanation]
Observations are the "raw" outputs of sensors, e.g., an image from a camera. An image doesn't necessarily provide a robot with information on where to, e.g., move next. States instead are derived from the sensor(s) data, and provide information that the robot can use to make a decision on what to do next to accomplish the goal, e.g., the position and orientation w.r.t. the road.
[explanation]

>> What is a traditional, or textbook, stateful autonomy architecture made of? <<

[ ] It depends on the organization designing these architectures.
[ ] Kalman filters, A*, and model predictive control (MPC).
[x] Perception, planning and control modules.
[ ] Sensorimotor connections.

[explanation]
Kalman filters, A*, and MPC are examples of particular algorithms used in estimation, planning and control respectively. Sensorimotor connections are not stateful, and different organizations don't redefine the textbook.
[explanation]

>> What is the main role of the Estimation block? <<

[ ] Smooth the observation data to mitigate unavoidable measurement noise.
[x] Transform sensor data into actionable information for the robot to accomplish its task.
[ ] Select which sensor data stream to utilize when multiple sensors are present.
[ ] Determine the probability of success of the robot's mission.

[explanation]
The estimation block contains some algorithm (many exist) to transform data from sensors to an estimate of information used by the planner and controller to accomplish their objectives.
[explanation]

>> Why is it a "belief"? <<

[ ] Given the user/designer determines the states, there is no way to know that the choice will be effective until testing it out.
[ ] Because states were introduced for organizational biases of big corporations, and everybody else believed it was a good idea. As it happens sometimes in the world of science, names just stuck around.
[x] The "real" value of states is unknown to the robot. The robot can only determine a "guess" based on available data. The belief is such a guess.
[ ] "Belief" is a variable with exact definition, but it hasn't been introduced to us yet.

[explanation]
The estimation block derives information, or tentative values for the state variables at each time instant, from the data made available from sensors. Real data is always noisy (has random fluctuations to the "real" value), and sometimes it might simply not contain enough information for the estimation algorithm to provide the "real" value. Estimators produce estimates, or "guesses," of the real values of state variables.
[explanation]

>> In the case of a Duckiebot driving in Duckietown, how many valid state variables choices could there be? <<

[ ] It's a 2D world, so 2.
[ ] It's a 2D world but the robot has a height, so 3.
[x] Infinite.
[ ] Zero, because Duckietown is a simplified representation of the real world.
[ ] It depends on the task.

[explanation]
Although not all choices of states are equally informative and efficient for computational reasons, there are always infinite choices of states for any controlled system.
[explanation]

>> What is the main role of the Planning block? <<

[ ] Make sure to compute the optimal strategy for achieving the given task to the controller block.
[x] Determine a nominal plan based on the robot's belief of the world, and readjust it on the fly as need be.
[ ] Inform the estimation block on which subset of the data to focus on.
[x] Act as "supervisor" to prevent undesirable outcomes, such as crashes in the case of self-driving.

[explanation]
The role of a planner is to provide a nominal plan, i.e., values for the state variables to be taken as ideal "references" from the controller, at every time instant.
[explanation]

>> What is the main role of the Control block? <<

[ ] Make sure the planner and estimators are efficiently executing their functions.
[ ] Decide on the best course of action to accomplish the given task.
[ ] Enable the robot sensors to collect more informative data as time goes by.
[x] Determine commands for the actuators to "fill the gap" between the plan and the belief of the robot.

[explanation]
The controller receives as input the state estimates (belief) and a nominal plan, and uses some algorithm (many exist) to compute commands for the actuators to drive the robot states towards the plan.
[explanation]

>> What is an "attention" mechanism? <<

[x] It's a way for information to flow "backward" in the feedback loop and increase overall performance by constraining the perception space.
[ ] It's a way to catch outliers in the state estimates and avoid catastrophic outcomes of the controller.
[ ] It's a way to debug complex software architectures.
[ ] It's a way to have the controller place the robot so the sensors can provide more informative data.

[explanation]
The planner could inform the perception module on what is most important to focus on.
[explanation]

>> Do you believe learning-based methods are superior to traditional architectures? <<

[x] Yes.
[x] No.
[x] I don't know, yet.

[explanation]
It depends on the task, the computational resources, the configuration of the plant, and many other variables. There is no clear-cut answer, but we are curious to know what you think!
[explanation]
