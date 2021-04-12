#!/usr/bin/env python
# coding: utf-8

# In[42]:


import numpy as np
# Heading control
def PIDController(
    v_0, 
    x_hat, 
    prev_e_x, 
    prev_int_x, 
    delta_t):
    """
    Args:
        delta_phi_right (:double:) delta phi right.
        delta_phi_left (:double:) delta phi left.
        prev_e (:double:) previous error.
        delta_t (:double:) delta time.
    returns:
        u (:double:) control command for omega.
        current_e (:double:) current error.
    """
    ## Set outside this function:
    # gain = 0.45
    # x0, y0, theta0 = [0, 0, 0]
    
    
    v_0=0.2

    # Error along x

    x_ref = 0.10

    # error
    e_x = x_ref - x_hat

    # integral of the error
    e_int_x = prev_int_x + e_x*delta_t

    # antiwindup
    e_int_x = max(min(e_int_x,3),-3)

    # derivative of the error
    e_der_x = (e_x - prev_e_x)/delta_t

    # PID parameters

    Kp_x=.8
    Ki_x=0.002
    Kd_x=3
    
    # PID controller for omega
    omega = Kp_x*e_x + Ki_x*e_int_x + Kd_x*e_der_x
    
    u = [v_0, omega]
    
    print(f"\n\nDelta time : {delta_t} \nE : {e_x} \nE int : {e_int_x} \nPrev e : {e_der_x}\nU : {u} \nX_hat : {x_hat} \n")
    
    return u, e_x, e_int_x

