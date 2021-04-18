#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np


# In[1]:


# TODO: write the PID controller function for y postion control.

# **DO NOT CHANGE THE NAME OF THE FUNCTION or its inputs**, it will break things

def PIDController(
    v_0, 
    y_ref, 
    y_hat, 
    prev_e_y, 
    e_int_y_prev,  
    delta_t):
    
    """
    Args:
        v_0 (:double:) speed (assume given).
        y_ref (:double:) reference lateral position (assume given)
        y_hat (:double:) the current estimated y coordinate.
        prev_e_y (:double:) previous tracking error.
        e_int_y_prev (:double:) previous integral error.
        delta_t (:double:) delta time.
    returns:
        u (:double:) car control command, includes v0 (const) and omega.
        current_e (:double:) current error.
        current_int_e (:double:) current integral error.
    """
    
    # TODO: write your PID controller here

    # PID parameters

    Kp_y = None
    Ki_y = None
    Kd_y = None
    
    # PID controller for omega
    omega = None
    
    u = [v_0, omega]
    
    return u, e_y, e_int_y

