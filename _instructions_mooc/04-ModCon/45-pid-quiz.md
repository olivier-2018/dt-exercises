PID Control
=====

>> How do you make a brick wall fly? <<

[ ] I didn't get it before, I don't get it now.
[ ] Brick walls stand or fall, they don't fly.
[ ] By dividing it by a brick wall and multiplying by an airplane.
[x] By dividing it by exactly the same brick wall and multiplying by an airplane.

[explanation]
The flying brick wall example is an attempt at showing the power of feed-forward control, a technique based on plant inversion. In theory, (i.e., mathematically) it can produce miracles like the above. In practice, all models.. (you should know how this phrase ends by now).
[explanation]

>> How are brick walls related to PID control? <<

[ ] The instructor is trying to be funny.
[x] Making the point that model-based controller design is a desirable approach.
[x] Making the point that PID control is not the best approach to control a plant.
[ ] Making the point that aerospace engineering uses more advanced control systems than the self-driving car industry.
[x] Making the point that there is a difference between theory and practice.

[explanation]
Feedforward control, ideally, is the best control approach. Unfortunately, more often than not, it does not work in the real world and is, at most, used in conjunction with feedback controllers. PID happens to not even be the second-best control approach, which is state feedback. They why PID?
[explanation]

>> Why PID control? <<

[x] It is popular.
[x] It is intuitive.
[x] It is simple to implement and tune.
[x] It is computationally efficient.
[x] It is the first thing to try if you are a control systems person. Never over engineer solutions!
[x] It works (well enough)!
[ ] With enough patience, optimal performance can be achieved.
[ ] With enough patience, constraints are guaranteed to be respected.
[ ] It never requires a model of the controlled plant.
[x] It can be implemented without a model of the plant.

[explanation]
PID control is the most popular controller for many good reasons. It just happens not to be the "best" (most resrouce efficient, most performant, most robust, etc.) controller, especially when the controlled systems become more complex (MIMO, nonlinear, coupled, etc.).
[explanation]

>> For what reason are we not using the plant model to design our PID controller? <<

[x] Because the math entry level is higher.
[x] Because it is not necessary to achieve sufficiently good performance, given a simple task.
[ ] Because we don't have enough insight on the plant's behavior.
[ ] Because there are no ways to tune PID coefficients based deductively.

[explanation]
We discuss a sequence of approaches to solving tasks of increasing complexity. If the task defined is to, e.g., drive a robot straight, there is no need to overcomplicate things. For a more formal approach to PID control design, we would need to introduce concepts like linear time-invariant (LTI) system, state-space representations, transfer functions, harmonic analysis, and loop shaping. For those of you interested in delving deeper, check out the provided "Thinking Activities" section of this module!
[explanation]

>> P, I, and D, to what? <<

[ ] to the reference signal provided by the planner.
[ ] to the state estimate.
[x] to the tracking error.
[ ] to the sensor measurements.

[explanation]
Proportional, integrative, and derivative control terms are so of the tracking error, which is defined as the difference between the reference signal and the measurements of the controlled variable.
[explanation]

>> What is the typically expected outcome of increasing the P term of the PID controller?<<

[x] Increased chance of instability.
[ ] Suppression of measurement noise.
[x] More aggressive response.
[ ] Higher error after "infinite" time.
[x] Higher overshoot.

[explanation]
"The further away, the harder it steers." As the proportional term increases, the typical outcomes are reduced time to target (rising time), reduced steady-state error, and a more reactive system response. Measurement noise will be directly amplified and injected into the feedback loop, causing undesirable effects.
[explanation]

>> What is the typically expected outcome of increasing the D term of the PID controller?<<

[x] Increased stability.
[x] Lower overshoot.
[ ] Better long-term accuracy of the response.
[ ] Suppression of measurement noise.

[explanation]
The derivative term allows the controller to approximate the future error, and compensate for it. The general result is the mitigation of excess behaviors, e.g., overshoot. The D component of a PID control signal tends to dampen the system's response, increasing stability, but reducing the system's responsiveness. The nemesis of D control is measurement noise, which can potentially generate D signals with "infinite" magnitude.
[explanation]

>> What is the typically expected outcome of increasing the I term of the PID controller?<<

[ ] Lower overshoot.
[ ] Faster system reaction times.
[x] Disturbance rejection.
[ ] Increased stability.

[explanation]
The I term allows the controller to take into account the history of the tracking error, allowing to compensate for biases (too high, too low). Integral control has the potential to eliminate steady-state error (in some systems) and most importantly brings to the table "disturbance rejection", i.e., the ability to recover from external unexpected perturbations to the system's behavior.
[explanation]

>> How many industrial applications do you think implement some form of PID control, today? <<

( ) Less than 50%.
( ) Between 50% and 60%.
( ) Between 60% and 75%.
( ) Between 75% and 90%.
(x) More than 90%.
( ) More than 95%.


[explanation]
We're not sure about the correct answer as of today, but it is "common knowledge" (and reported in the 2001 paper below) that >90% of industrial automation applications are controlled through PID, or variations thereof. Some sources report up to 97%. Although the data is a little old (and difficult to verify), it is safe to take away that the great majority of real-world control problems are solved through PID.  

K.J. Åström, T. Hägglund, The future of PID control, Control Engineering Practice, Volume 9, Issue 11, 2001, Pages 1163-1175, ISSN 0967-0661, https://doi.org/10.1016/S0967-0661(01)00062-4.
[explanation]

>> Finally: which one is better, Star Wars or Star Trek? <<

(x) Star Trek
(x) Star Trek

[explanation]
Star Trek is like math. The only reason one can't fall in love with it is because it is not well understood. The only reason for which this question is here is because Andrea Censi should to spend more time watching Star Trek, and less playing with robots! Live long and prosper.
[explanation]
