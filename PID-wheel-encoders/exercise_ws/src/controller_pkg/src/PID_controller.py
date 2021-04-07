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

    # x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
    # y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
    # theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/(2*L)
    e = delta_phi_right-delta_phi_left # the two wheels have to spin the same
    Kp=1
    Ki=0
    Kd=0

    u = Kp*e + Ki*(prev_e+e)*delta_t + Kd*e/delta_t
    
    return u, e

