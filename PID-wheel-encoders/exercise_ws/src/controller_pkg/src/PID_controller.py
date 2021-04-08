#!/usr/bin/env python
# coding: utf-8

# In[ ]:



def PIDController(delta_phi_right, delta_phi_left, prev_e, delta_t):
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

    e = delta_phi_left-delta_phi_right # the two wheels have to spin the same
    Kp=1
    Ki=0
    Kd=0

    u = Kp*e + Ki*(prev_e+e)*delta_t + Kd*e/delta_t

    print(f"\n\nDelta time : {delta_t} \nE : {e} \nU : {u} \nDelta phi right : {delta_phi_right} \nDelta phi left : {delta_phi_left}")

    
    return u, e

