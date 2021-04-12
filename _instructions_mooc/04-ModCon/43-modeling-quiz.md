Modeling of a differential drive robot
=====

>> What is the purpose of mathematical modeling? <<

[ ] To truthfully represent the real physical processes ongoing in the system.
[x] To predict the future behavior of the system.
[x] To understand the influence of various parameters on the variables of interest to us.
[x] To obtain an actionable mathematical infrastructure to analyze and solve problems.

[explanation]
"All models are wrong, but some are useful." (-- George Box, 1919-2013). The point of modeling is to obtain something useful, not to describe the truth. Nobody knows what the truth is. All physics is about creating frameworks that justify observations. All is "as if" the framework were a truthful representation, but nobody knows exactly what is going on.
[explanation]

>> What is a differential drive vehicle?<<

[ ] A vehicle with a swappable gearbox.
[ ] A vehicle with four controlled wheels, two of which push, two of which steer.
[x] A vehicle with two controlled wheels, each of which is driven independently.
[ ] A vehicle with four-wheel drive.

>> What kind of model have we derived for our Duckiebot?  <<

[ ] A mathematical model.
[ ] A learning-based model.
[x] A kinematic model.
[ ] A dynamic model.

[explanation]
We have derived a kinematics model for a differential drive vehicle. Kinematics studies the relation of position, velocity, and accelerations of geometric points. This means we have purposefully ignored the vehicle dynamics. A dynamic model would have included considerations on the mass of the vehicle, and forces and torques at play.  
[explanation]

>> Why is the kinematics model of the Duckiebot wrong? <<

[x] The Duckiebot has a mass!
[x] The Duckiebot has friction(s)!
[x] The Duckiebot is not symmetric!
[ ] The Duckiebot is not a differential drive vehicle!

[explanation]
To keep the model simple, we have neglected many aspects of the physical processes going on. Are you driving on different surfaces, more or less bumpy? Not included. Does your omnidirectional wheel create friction? Yes, but not considered. Do your wheels slip? _Reduce the wheel gain_!
[explanation]

>> Why is the kinematics model of the Duckiebot useful? <<

[x] It captures the fundamental input-output behavior of the vehicle.
[x] It offers guidelines on factors that will influence the driving experience.
[ ] It allows us to improve the Duckiebot design.
[ ] It is the most realistic model we could have come up with.
[x] It is a good compromise of simplicity and descriptive power.

[explanation]
The kinematic model captures the essence of the vehicle driving experience given the tasks we intend to accomplish in this course: lane following and obstacle avoidance. It tells us which parameters play a significant role (R, L, motor coefficients) and informs on what to do (and not) to, e.g., make sure the assumptions are respected.
[explanation]


>>  What is the input to the forward kinematics model? <<

[x] Angular velocities of the wheels.
[ ] Wheel radii and baseline.
[ ] Desired robot linear and angular speed.
[ ] The pose of the robot.

>>  What is the output of the inverse kinematics model? <<

[ ] Angular velocities of the wheels.
[ ] Wheel radii and baseline.
[x] Desired robot linear and angular speed.
[ ] The pose of the robot.

>> What is a practical implication of the no slipping assumption made? <<

[ ] Only drive on the Duckietown road tiles.
[ ] Change the robot wheels so it doesn't slip on any surface.
[x] Lower the robot's speed until it doesn't slip.
[ ] Drive as fast as you can and try to "break" the model.

[explanation]
While the slipping indeed is a function of the materials of the wheel and the surface on which we are driving, the best thing we can do to make our model still preserves predictive power is to lower the speed of the robot until the wheels do not slip.
[explanation]

>> Looking at the model, what is the expected outcome of having a loose battery on the Duckiebot? <<

[ ] The robot will go unstable.  
[ ] The robot will not drive straight.
[x] We are unable to tell.
[x] The center of mass of the robot will change over time.

[explanation]
The battery is the component with the highest mass of the robot. If it were loose, and move during driving, it would be equivalent to violating assumption 2 of our model (\dot c = 0). This means that the presented equations would no longer hold, hence they would fail to provide insight on expected outcomes.
[explanation]

>> What is missing in the robot model? <<

[x] We did not model the sensors.
[x] We did not model the PWM signals!
[x] We did not model the effect of battery charge.
[ ] We did not model backward driving.

[explanation]
In the "big picture slide" (robot vs. agent) we called "plant" the serial connection of actuators, actual chassis configuration, and sensors. We have not talked about sensors, yet, as they will be covered in the next modules. We moreover neglected the translation of voltage to motors to PWM (Pulse Width Modulation) signals, although it indeed happens on the Duckiebot. This "translation" is at the motor driver's level - what we consider "low level", and just assume it will work well enough.  
[explanation]

>> What is the forward kinematics particularly useful for? <<

[ ] To fix the Duckiebot.
[ ] To drive the Duckiebot.
[x] To predict the future pose of the Duckiebot.
[x] To predict the future pose of Duckies on the road.

[explanation]
The forward kinematics model provides the variation of the pose in time when commands are sent to the wheels. This can be used to "forward propagate" the state estimate in time and is therefore particularly useful for estimation. Given Duckies in Duckietown don't move (yet!), the Duckiebot's forward kinematics can be used to infer the variation of the relative pose   
[explanation]

>> What is inverse kinematics particularly useful for? <<

[ ] To fix the Duckiebot.
[x] To drive the Duckiebot.
[ ] To predict the future pose of the Duckiebot.
[ ] To predict the future pose of Duckies on the road.

[explanation]
The output of a controller module will typically be desired vehicle linear and angular speeds. The inverse kinematics will tell us which commands to send to the wheels to achieve that desired outcome.
[explanation]

>> If your robot turns "on the spot", what is happening? <<

[ ] The same command is being sent to both wheels.
[x] The wheels are moving at equal but opposite speeds.
[ ] One of the wheels is receiving a zero velocity command.
[ ] The instantaneous center of curvature is moving on an n-dimensional hyperplane, where n is the number of duckies in line of sight.

[explanation]
Once the kinematics model assumptions are respected, turning on the spot means the center of instantaneous curvature of the robot coincides with the origin of the robot frame (point A), i.e., d = 0, which is possible only when the wheel speeds are equal in magnitude but opposite in sign.
[explanation]

>> What practical information do we need to use this model in the real world?<<

[x] The baseline (L).  
[ ] The wheel radii (R_left, R_right).
[ ] The omnidirectional wheel friction coefficient (k_omni).
[x] The wheel radius (R).
[x] The motor constants.

[explanation]
We need some real-world numbers to use the model. Our model is parametrized by L, R, and the motor constants.
[explanation]
