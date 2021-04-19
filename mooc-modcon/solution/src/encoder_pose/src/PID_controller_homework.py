#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np

# write the PID controller function for y postion control.

def PIDController(
    v_0,
    y_ref,
    y_hat, 
    prev_e_y, 
    prev_int_y, 
    delta_t):
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        u (:list:) 1x2 array of commands for the Duckiebot: [v0, omega] 
        current_e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        current_int_e (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # error
    e_y = y_ref - y_hat

    # integral of the error
    e_int_y = prev_int_y + e_y*delta_t  
    
    # antiwindup
    e_int_y = max(min(e_int_y,0.5),-0.5)

    # derivative of the error
    e_der_y = (e_y - prev_e_y)/delta_t

#     # PID parameters sim (v0 = 0.2, yref = 0.2 then -0.1)
#     Kp_y= 9
#     Ki_y= 0.02
#     Kd_y= 120 

    # PID parameters robot (v0=0.2, yref = 0.1 then -0.2)
    Kp_y= 9
    Ki_y= 0.05
    Kd_y= 12
    
    # PID controller for omega
    omega = Kp_y*e_y + Ki_y*e_int_y + Kd_y*e_der_y
    
    u = [v_0, omega]
    
    # Debugging 
    # print(f"\n\nDelta time : {delta_t} \nE : {e_y} \ne_int : {e_int_y} \ne_der : {e_der_y} \nU : {u} \ny_hat: {y_hat} \ny_ref: {y_ref}")

    
    return u, e_y, e_int_y

