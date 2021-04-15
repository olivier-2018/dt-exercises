#!/usr/bin/env python
# coding: utf-8

# In[20]:


def DeltaPhi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg (ROS encoder message)
            prev_ticks (Current ticks)
        Return:
            rotation_wheel (double) Rotation of the wheel
            ticks (int) current number of ticks
    """
    ticks = encoder_msg.data

    delta_ticks = ticks-prev_ticks

    N_tot = encoder_msg.resolution

    alpha = 2*np.pi/N_tot

    rotation_wheel = alpha*delta_ticks
    
    return rotation_wheel, ticks


# In[5]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your odometry function! 

import numpy as np 

# DO NOT CHANGE THE NAME OF THIS FUNCTION OR THINGS WILL BREAK

def poseEstimation( R, # radius of wheel (assumed identical)
                    baseline_wheel2wheel, # distance from wheel to wheel (center); 2L of the theory
                    x_prev, # previous estimate - assume given
                    y_prev, # previous estimate - assume given
                    theta_prev, # previous estimate - assume given
                    delta_phi_left, # previous estimate - assume given
                    delta_phi_right):
    
    """
        Calculate the current Duckiebot pose using dead reckoning approach.

        Returns x,y,theta current estimates:
            x_curr, y_curr, theta_curr (:double: values)
    """
    
    # x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
    # y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
    # theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/baseline_wheel2wheel
    
    w = [R, R / baseline_wheel2wheel, 1]
    x = np.array(
        [
            [
                (delta_phi_left + delta_phi_right) * np.cos(theta_prev) / 2,
                (delta_phi_left + delta_phi_right) * np.sin(theta_prev) / 2,
                0,
            ],
            [0, 0, (delta_phi_right - delta_phi_left)],
            [x_prev, y_prev, theta_prev],
        ]
    )
    x_curr, y_curr, theta_curr = np.array(w).dot(x)

    return x_curr, y_curr, theta_curr

