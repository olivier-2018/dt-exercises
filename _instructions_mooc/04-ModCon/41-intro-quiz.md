Control Systems basics
=====

>> What do we mean by using the term "Control Systems"? <<

[ ] The perception module
[ ] The planning module
[x] The controller module
[x] The perception, planning, and control modules

[explanation]
For historical reasons, there is some ambiguity on the use of the term "control systems" (or just "controls"). Technically it would relate only to the design of the actual controller block, but it has become to be a synecdoche for the whole agent design.
[explanation]


>> Why bother designing a controller? <<

[ ] Inner desire of humans to dominate machines.
[x] To have systems behave how a user wants, rather than following their natural dynamics.
[ ] Because everybody does it.
[ ] Because I am told to do so.

[explanation]
Systems have their natural dynamics, fundamentally dictated by the laws of physics. More often than not, just "letting the system go" will not meet the user's requirements. Control systems leverage our abilities to _measure_ quantities of interest and to _actuate_ (or influence the physical world through devices), to drive the system where we want, and how we want.
[explanation]

>> Assuming we can measure quantities of interest and drive the system through actuators, are all systems controllable? <<

[ ] In principle, yes, but some will be (significantly) more complicated and costly than others, so in practice not always.
[ ] In principle yes, but only through more advanced controllers (not covered in this course).
[ ] No, because we have not discovered sufficiently advanced controllers yet. AI might solve this challenge.
[x] No, because some systems simply can't be controlled.
[ ] It depends on our understanding/ability to model the system mathematically.

[explanation]
"You can't make the Enterprise out of a car."

There are intrinsic limitations to our ability to drive a given system's evolution in time. It depends on the choice of actuators (e.g., let's add a warp drive to that sedan!), as well as the _system's behavior_. The latter is given by the laws of physics. In the example above, we would first need to install inertial dampeners to our car to make things happen (which would imply having to change the laws of physics).

This notion is technically known as _controllability_.        
[explanation]

>> Ok, but _in practice_, what is a controller / how does it look like? <<

[x] Lines of code.
[x] One or more mechanical devices.
[ ] Human beings doing things.

[explanation]
A controller is typically a _logic_, that outputs _decisions_. These decisions are translated to the real world through actuators. It is actually possible to create control logics with analog devices too (see example below).
[explanation]

Control Objectives
=====

>> Stability is the first design objective for any controller. Why? <<

[x] Unstable systems are potentially unsafe.
[ ] Unstable systems are uncontrollable.
[x] Unstable systems can break down.
[ ] Unstable systems don't exist, because there is no infinite energy in the universe.
[x] Not all systems should be controlled with stability as an objective.

[explanation]
Once we all agree that controls is about making systems behave as we want, there are very few instances in which sending a system unstable might be desirable (exceptions being, e.g., acrobatic flight or selected military/destructive applications). An unstable system will behave unpredictably, potentially with catastrophic results.
[explanation]

>> How would you imagine, in practice, a car (or a Duckiebot) going unstable? <<

[x] Driving outside of the road (without heading back).
[x] Flipping over.
[ ] Crashing in an obstacle.
[ ] Engine not starting.

[explanation]
According to the definition of stability provided (BIBO stability), a system goes unstable when an output of the system diverges (always continues growing in time), when provided with a finite energy input. When defining the output as "distance from the center of the lane", driving outside of the road would be an example of instability. If the roll angle w.r.t. the local horizontal is taken as an example, a car losing contact with the ground and "flipping over", would be another example of an unstable system.  
[explanation]

>> Which of the following are good examples of performance criteria for controller design?<<

[x] Disturbance rejection: the ability to compensate for external stimuli.
[x] Noise attenuation: the ability to minimize the effect of high-frequency signals added to the measurements or inputs.
[x] Time before reaching the target state.
[x] Tracking error: how closely a reference signal is followed.

[explanation]
These are all examples of good performance criteria. Each could be formalized and quantified, but it would require a mathematical background outside the scope of this course. Please visit the "Thinking activities" module to learn more about this (in particular, the Controls I course).
[explanation]

>> What does a robust controller guarantee? <<

[ ] That controlled systems never have catastrophic events that compromise their ability to operate.
[ ] Stability and performance regardless of the choice of sensors.
[x] Stability and performance even in the presence of model uncertainty.
[ ] Stability and performance despite limited access to resources (e.g., the energy available).

[explanation]
Model uncertainty is defined as a "bounded variation" of the parameters describing the controlled system's model. Choosing completely different sensors (e.g., a camera instead of the planned pressure sensor for altitude control of a hot air balloon) might induce a completely different structure in the plant's model, which wouldn't not therefore be a "bounded" variation.
[explanation]

>> What could count as "bounded variation" in the system's behavior of a Duckiebot?  <<

[x] Wear and tear of components over time.
[x] Slight imperfections in the assembly.
[ ] Leaving the lid on the camera.
[ ] Not placing a Duckie on top of the Duckiebot while driving.

[explanation]
When it comes to Duckies, we lose it. Duckies are non-functional _yet essential_ to get everything working!
[explanation]

Open vs. closed-loop control
=====

<img src="/static/fireplace.jpg" style="width: 40%" alt="fireplace"/>

>> What control architecture is used to regulate this system's output (heat generated)? <<

[ ] Closed-loop
[x] Open loop
[x] We cannot tell from this picture.
[ ] This system is not controlled.

[explanation]
To keep the fire going (and generate heat), one needs to add, e.g., wood. Typical fireplaces are open-loop controlled, as wood is added manually. There is a slim chance that a mechanical system not in the picture is throwing wood in the fireplace once the temperature in the room is too high.
[explanation]

<img src="/static/saloon-door.jpg" style="width: 40%" alt="fireplace"/>

>> What control architecture is used to regulate this system's output (rest position of the door)? <<

[x] Closed-loop
[ ] Open loop
[x] We cannot tell from this picture.
[ ] This system is not controlled.
