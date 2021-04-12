Robot Representations
=====

>> Why are robot representations important? <<

[x] To determine performance metrics in accomplishing tasks.
[x] To formally define tasks.
[x] For robots to quantify their relationship to the world around them.
[ ] To have robots communicate with each other.

[explanation]
Autonomous robots accomplish tasks. Robots need to be able to estimate how successfully (or not) they are doing, so to attempt to do better. A choice of representation provides the framework into which to define, quantify and eventually evaluate the performance of the robot.
[explanation]

>> Where do robots get representations from? <<

[ ] From the planning module.
[x] From the designer.
[ ] From the observations.
[ ] From the model of the controlled plant.

[explanation]
Which representation best suits a given task is typically determined by the designer, as they belong to the agent side of the autonomy loop.
[explanation]

>> What are examples of valid representations in the Duckietown world? <<

[x] Spherical topography, GPS coordinates.
[x] Duckietown-tile gridmap (each city tile is a cell).
[x] Road network graph (nodes are intersections, edges are drivable roads).
[x] Feature-based map, where features could be line segments, traffic signs, etc.

[explanation]
All these representations might be useful for different tasks. Not all of these examples would work equally well for a given task. E.g., GPS localization might not be sufficiently accurate to allow a Duckiebot to avoid a Duckie on the road.
[explanation]

>> What do all representations have in common? <<

[ ] The computational efficiency.
[ ] The perception algorithms used.
[ ] The planning algorithms used.
[x] The need for a reference frame.

[explanation]
Representations require defining a reference system, whether the robot, the world reference frame, Greenwich meridian, etc.
[explanation]

>> How do we quantify representations? <<

[x] With state variables.   
[ ] With numbers.
[ ] With models.
[ ] With algorithms.

>> What does it mean for a state to be Markov? <<

[ ] Robots using that parametrization will successfully accomplish their tasks.
[x] Estimating present states can be done with minimal memory and computation.  
[x] The values of the state variables at present completely summarize the history of the state.
[x] The future state variables are independent of the past, given the present.

[explanation]
The Markov assumption implies that the present completely summarizes the history of inputs and evolution of the state., so it's no longer needed to keep memory of it.
[explanation]

>> What is a good choice of state variables in Duckietown?<<

[x] The pose: position and orientation of the robot w.r.t. the world.    
[ ] The Special Orthogonal group SO(2).
[ ] The Special Euclidean group SE(2).
[ ] The pose: position, orientation, speed, and angular velocity of the robot w.r.t. the world.

>> Is the true pose of, e.g., a Duckiebot at a given instant unique?<<

[ ] Yes
[x] No
[ ] It's impossible to tell with the provided information.

[explanation]
The pose cab calculated w.r.t. different reference frames, e.g., a fixed world frame or the robot frame, which is "attached" to the robot and moves with it. A Duckie on the road, therefore, might be at a certain pose w.r.t. the Duckiebot, but a different one w.r.t. the world frame.
[explanation]

>> What is special about the rotation matrix?<<

[ ] It is particularly symmetric.
[ ] It is particularly square.
[x] It is particularly easy to invert.
[ ] It is particularly easy to calculate.

[explanation]
Rotation matrices are orthogonal, which implies that their inverse is equal to their transpose. This makes them easy to invert.
[explanation]

>> How can we express the pose of any point in the world?<<

[ ] Through elements of the special Orthogonal group.
[ ] Through rotations.
[x] Through elements of the special Euclidean group.
[ ] Through translations.

[explanation]
The special Euclidean group includes matrices, that include in their structure both rotations matrices and translation vectors, i.e., all the information needed to express poses w.r.t. different reference frames.
[explanation]

>> Why do we introduce SE(2)?<<

[ ] For the sake of completeness.
[ ] To provide a structure to analyze n-dimensional representations.
[ ] Because they're the only way to express poses w.r.t. different reference frames.
[x] Because its elements allow to "move between frames" with matrix multiplication operations.

[explanation]
Matrix multiplications are mathematical structures that can be treated efficiently. For example, relations can be inverted easily and "daisy-chained" intuitively.
[explanation]
