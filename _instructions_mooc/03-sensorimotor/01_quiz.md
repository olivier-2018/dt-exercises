>> You are tasked to design a robot. Where to start from? <<

[ ] Study control systems.
[ ] Study AI.
[x] The task(s), or objectives the robot must complete.
[ ] The environment(s) the robot will operate in.
[ ] The hardware.

[explanation]
All the above are important in designing a robot. But knowing the task first will inform the design of the other components. The environment(s) the robot will operate in is a close second. The hardware comes next. We can then move to designing the agent, and evaluate the efficacy of different approaches to solving the task, given a plant.  
[explanation]


>> What does "no robot is an island" mean?<<


[ ] Robots operate best when operating in the presence of other robots.
[ ] Robots don't fare well with water.
[x] Roboticists should consider the physical operating environment in addition to the robot itself.
[ ] It is not possible to design a functioning robot without building upon a previous one.

[explanation]
The key takeaway of "no robot is an island" is that the environment in which robots operate, including humans, if any, plays a critical role in the design process. Structuring the environment, when possible, make the Roboticist's life so much easier.
[explanation]

>> What do we mean by "design line"? <<


[ ] The distinction between what is hardware and what is software.
[ ] The distinction between what is fixed and what is variable.
[x] The distinction between the plant and the agent of a robot.
[ ] The limit each Roboticist should keep in mind when considering the repercussions of their work on society.
[ ] The aesthetical form of the robot.

[explanation]
Although Duckiebots are extensible, as in additional sensors and/or actuators can be added to the standard configuration, a choice has already been made and provided to you, both for the robot, and its operating environment. The design line, therefore, identifies the boundary between the given plant and the agent that is to be designed to achieve autonomous behaviors.
[explanation]


>> What is one of the biggest challenges in robotics? <<

[ ] Installing Ubuntu
[ ] Assembling the hardware, especially if it has small screws
[x] Harmonious co-design of hardware, software, and environment, for the purpose of a specific task
[ ] Configuring the Wifi network

[explanation]
Biologists say that "everything depends on everything". While robots are arguably less complex than most living beings, the system-level interactions between the various modules and the environment represent the biggest challenge in modern autonomy. Hardware can be considered a solved problem.   
[explanation]

>> What is an "agent" in robotics? <<

[ ] The software development infrastructure used to create the code that runs on any robot.
[ ] The mind of the Duckiebot.
[ ] The mind of any robot.  
[x] The process that decides what commands to send to actuators based on the observations from the sensors.


[explanation]
The agent indeed can be considered the "mind" of any robot, but robots don't really have a mind. They do have a more or less complex infrastructure that allows to transform the data coming from the sensors (observations) into signals (commands) for the robot actuators.
[explanation]

>> What is a "plant" in robotics? <<

[ ] Same as all plants. They are alive, but are not animals. They're typically green-ish and have foliage, although many varieties exist.
[ ] The code used to transform commands sent to a robot's actuators into sensor observations.
[ ] The mathematical representation we use to describe the behavior of the hardware.
[ ] The laws of physics.
[x] The robot hardware configuration and the environment it operates.

[explanation]
"Plant" is a typical control systems term originally used to describe the physical process one seeks to regulate. By extension, the term "plant" - often ambiguously called "system" or "process" - encompasses the hardware configuration, definition of inputs and output signals and physical limitations dictated by the hardware configuration, the environment it operated, and the applicable laws of physics.   
[explanation]

>> What is an "actuator"? <<

[x] Any device that makes the robot move.  
[x] Any device that allows the robot to interact with the physical world.
[ ] Any device that makes the robot communicate with other robots.
[ ] Any device that makes the robot take better decisions autonomously.

[explanation]
Without actuators, robots cannot act!  
[explanation]

>> What is a "sensor"? <<

[ ] Any device that allows a robot to perceive its internal states.
[ ] Any device that allows a robot to perceive the environment around it.
[ ] Any device that enables the robot to refine its nominal plan to achieve the given task.
[x] Any device that provides measurements of some physical quantity (light, pressure, accelerations, etc.).

[explanation]
Sensors enable robots to perceive the world around them, in some way.  
[explanation]

>> What is the best approach to designing a robot agent? <<

[ ] Deep Learning
[ ] Artificial Intelligence
[ ] Machine Learning
[ ] Control Systems
[x] All of the above  
[ ] None of the above
[x] Nobody really knows: it's an ongoing field of scientific research
[ ] Prayers

[explanation]
Robot autonomy is an ongoing field of research, so nobody really knows what the "ultimate solution" is. To date, we know that complex real-world problems, like having a car driving autonomously in an unstructured environment, uses a mix of different approaches.
[explanation]
