#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
# Heading control

# v_0=0.15 # TODO: Vincenzo make it tunable online kind of like rosparam set
# theta_ref = 70*np.pi/180 # TODO: Vincenzo make it tunable online kind of like rosparam set


def PIDController(v_0, theta_hat, prev_e, prev_int, delta_t): #add theta_ref as input
    """
    Args:
        delta_phi_right (:double:) delta phi right.
        delta_phi_left (:double:) delta phi left.
        prev_e (:double:) previous error.
        delta_t (:double:) delta time.
    returns:
        u (:double:) control command for omega and v_0 (constant).
        current_e (:double:) current tracking error.
    """
    
    # Constant linear velocity of the robot
    v_0 = 0.2 # TODO: Vincenzo make it tunable online kind of like rosparam set

    # Reference angle in radians
    theta_ref = 70*np.pi/180 # TODO: Vincenzo make it tunable online kind of like rosparam set
    
    # Tracking error
    e = theta_ref - theta_hat

    # integral of the error
    e_int = prev_int + e*delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)


    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    Kp = 1
    Ki = 0.0
    Kd = 0.1

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der
    
    u = [v_0, omega]
    
    # print(f"\n\nDelta time : {delta_t} \nE : {e} \nE int : {e_int} \nPrev e : {prev_e}\nU : {u} \nTheta : {theta_hat} \n")

    
    return u, e, e_int


## Exercise: lateral position control 

# v_0 = u[0]
# rosparam set /!hostname/kinematics_node/gain v_0


# for the exercise: no longer heading control, but y direction control

# challenge used to evaluate is LF
# world is a straight lane (or whatever)
# initial condition of the robot is in the opposite lane
# (we are playing now with (d, theta))
# d0 = -2 (whatever brings you in the opposite lane)
# theta0 = whatever = 0
# d_ref = 0 # (this means be at the center of the lane, as in regular lane following)
# e = d_ref - dhat
# (dhat is initialized with d0)
# dhat = dlast + delta_d
# theta = R/(2*L)*(delta_phi_right-delta_phi_left)
# delta_d = R/2*(delta_phi_l + delta_phi_r)*cos(theta) #(double check cos)
# v = gain = 0.5 (whatever)
# omega = PID(e)
# u = [v omega]

