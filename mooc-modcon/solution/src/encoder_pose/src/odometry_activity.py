#!/usr/bin/env python
# coding: utf-8

# In[1]:


def DeltaPhi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg (ROS encoder message)
            prev_ticks (Current ticks)
        Return:
            rotation_wheel (double) Rotation of the wheel
            ticks (int) current number of ticks
    """
    ticks = msg_encoder.data

    delta_ticks = ticks-prev_ticks

    N_tot = msg_encoder.resolution

    alpha = 2*np.pi/N_tot

    rotation_wheel = alpha*delta_ticks
    
    return rotation_wheel, ticks


# In[1]:



import numpy as np 

# DO NOT CHANGE THE NAME OF THIS FUNCTION
def poseEstimation( R, # radius of wheel (assumed identical)
                    baseline_wheel2wheel, # distance from wheel to wheel (center); 2L of the theory
                    x_prev,
                    y_prev,
                    theta_prev,
                    delta_phi_left,
                    delta_phi_right):
    """
        Calculate the current Duckiebot pose using dead reckoning approach.

        Returns x,y,theta current estimates:
            x_curr, y_curr, theta_curr (:double: values)
    """
    x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
    y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
    theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/baseline_wheel2wheel
    
    return x_curr, y_curr, theta_curr

