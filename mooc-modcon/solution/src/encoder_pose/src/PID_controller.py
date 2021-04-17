#!/usr/bin/env python
# coding: utf-8

# In[12]:


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
    v_0 = 0.5

    # Reference angle in radians
    theta_ref = 90*np.pi/180 # TODO: Vincenzo make it tunable online kind of like rosparam set

    # Tracking error
    e = theta_ref - theta_hat



    # integral of the error
    e_int = prev_int + e*delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)


    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    Kp = 10
    Ki = 0
    Kd = 0

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der

    u = [v_0, omega]

    # print(f"\n\nDelta time : {delta_t} \nE : {e} \nE int : {e_int} \nPrev e : {prev_e}\nU : {u} \nTheta : {theta_hat} \n")


    return u, e, e_int
