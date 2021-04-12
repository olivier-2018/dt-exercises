Odometry
=====

>> Why is odometry useful? <<

[ ] To enable Duckiebots to continuously measure their pose with respect to other objects in the world.
[x] To have the Duckiebot propagate its belief.
[x] To have the Duckiebot estimate its current and future pose.
[x] To enable Duckiebots to update the internal notion of their position in the world.

[explanation]
Odometry allows Duckiebots to estimate their future pose, given an initial condition and measurements from sensors. In the case of interoceptive sensors, e.g., wheel encoders, Duckiebots are "blind" to what happens in the world around them. Exteroceptive sensors, e.g., a camera, will be needed to introduce that capability.
[explanation]

>> What is a common problem of odometry?  <<

[x] The estimate of the pose drifts from the true pose.
[ ] It is computationally intensive and it introduces delays.
[ ] It requires having wheel encoders.
[ ] It is completely arbitrary because the initial pose is unknown.

[explanation]
The pose estimates provided by the odometry tend to drift from the true values, due to the dead reckoning algorithm utilized, as well as for systematic errors induced by the use of a motion model. Typically the initial pose is taken as (0,0,0).
[explanation]

>> How to mitigate the common practical challenges of odometry using wheel encoders? <<

[x] Do accurate system identification.
[x] Use a high update frequency.
[x] Use exteroceptive sensors to complement the estimate.
[x] Don't allow the wheels to slip!

[explanation]
All the above are good ways to mitigate the drift caused by using wheel encoders. Using exteroceptive sensors is a qualitative step forward, allowing the robot estimate to be "anchored" to features in the real world, somewhat resetting the drift. We will introduce cameras in the next module and see this at work.
[explanation]

>> What do wheel encoders do?  <<

[ ] Measure the wheel's traveled distance.
[ ] Measure the wheel's average speed in a certain time interval.
[x] Measure the motor rotation.
[ ] Measure the vehicle's angular speed.

[explanation]
Wheel encoders measure the rotation of the motor shafts. All other quantities are derived and subject to uncertainty induced by other factors: wheel radius for traveled distance, timestamp provided by the system to measure speed, etc.
[explanation]

>> Given a motion model and an initial condition, what is the first step to solving the odometry problem?  <<

[x] Placing a Duckie on the Duckiebot.
[ ] Project the vehicle's displacements in the world frame.
[ ] Infer the vehicle's linear and angular displacements.
[x] Count the ticks from the encoders and obtain the wheel displacements.

[explanation]
Obtain the measurements from the sensors and use the motion model to evaluate the wheel displacements. Needless to say, everything is invalidated if no Duckies are on board.
[explanation]

>> Given a motion model and an initial condition, what is the second step to solving the odometry problem?  <<

[x] Infer the vehicle's linear and angular displacements.
[ ] Count the ticks from the encoders and obtain the wheel displacements.
[x] Add more Duckies on the Duckiebot.
[ ] Project the vehicle's displacements in the world frame.

[explanation]
Using the kinematics model, we can relate the wheel displacements to the vehicle displacements. Adding more Duckies onboard is good, but don't overdo it. We don't want any accidents.
[explanation]

>> Given a motion model and an initial condition, what is the third step to solving the odometry problem? <<

[ ] Infer the vehicle's linear and angular displacements.
[x] Ensure all the Duckies on the Duckiebot are secured.
[x] Project the vehicle's displacements in the world frame.
[ ] Count the ticks from the encoders and obtain the wheel displacements.

[explanation]
Using the estimate of the robot's orientation with respect to the world frame, we can project the displacement in that reference frame and perform the pose update.
[explanation]
