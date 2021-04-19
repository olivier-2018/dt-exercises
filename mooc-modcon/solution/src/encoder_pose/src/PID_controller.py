#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np

# Heading control
# Do not change the name of the function, inputs or outputs. It will break things.

def PIDController(v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t): #add theta_ref as input
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        theta_ref (:double:) reference heading pose
        theta_hat (:double:) the current estiamted theta.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        u (:list:) 1x2 array of commands for the Duckiebot: [v0, omega] 
        current_e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        current_int_e (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # Tracking error
    e = theta_ref - theta_hat

    # integral of the error
    e_int = prev_int + e*delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)

    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    Kp = 5
    Ki = 0.2
    Kd = 0.1

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der
    
    u = [v_0, omega]
    
    # For debugging
    # print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n Abs e {np.rad2deg(abs(e - prev_e))}")
    
    return u, e, e_int

