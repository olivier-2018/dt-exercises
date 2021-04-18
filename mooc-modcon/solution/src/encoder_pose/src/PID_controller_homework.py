#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np


# In[1]:


# write the PID controller function for y postion control.

# **DO NOT CHANGE THE NAME OF THE FUNCTION**, it will break things

def PIDController(
    v_0,
    y_ref,
    y_hat, 
    prev_e_y, 
    prev_int_y, 
    delta_t):
    """
    Args:
        v_0 (:double:) speed (can be changed setting up a v_0 variable).
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) previous error along y with respect to the setpoint.
        prev_int_y (:double:) previous integral error term (useful fo the integral action).
        delta_t (:double:) delta time.
    returns:
        u (:double:) control command for omega.
        current_e (:double:) current error.
        current_int_e (:double:) current integral error.
    """
    
    # Error along x


    # error
    e_y = y_ref - y_hat

    # integral of the error
    e_int_y = prev_int_y + e_y*delta_t  
    
    # antiwindup
    e_int_y = max(min(e_int_y,1),-1)
    

    # derivative of the error
    e_der_y = (e_y - prev_e_y)/delta_t

    # PID parameters

    Kp_y= 2
    Ki_y= 0.3
    Kd_y= 100
    
    # PID controller for omega
    omega = Kp_y*e_y + Ki_y*e_int_y + Kd_y*e_der_y
    
    u = [v_0/2, omega]
    
    #print(f"\n\nDelta time : {delta_t} \nE : {e_y} \nE int : {e_int_y} \nPrev e : {e_der_y}\nU : {u} \nX_hat : {y_hat} \n")
    print(f"\n\nDelta time : {delta_t} \nE : {e_y} \ne_int : {e_int_y} \ne_der : {e_der_y} \nU : {u} \ny_hat: {y_hat} \n")

    
    return u, e_y, e_int_y

