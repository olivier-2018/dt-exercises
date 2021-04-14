#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as np


# In[ ]:


def DeltaPhi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg (ROS encoder message)
            prev_ticks (Current ticks)
        Return:
            rotation_wheel (double) Rotation of the wheel
            ticks (int) current number of ticks
    """
    #rotation_wheel = ticks = 0
    
    ticks = encoder_msg.data

    delta_ticks = ticks-prev_ticks

    N_tot = encoder_msg.resolution

    alpha = 2*np.pi/N_tot

    rotation_wheel = alpha*delta_ticks
    
    return rotation_wheel, ticks


# In[ ]:



import numpy as np # already imported above 

# DO NOT CHANGE THE NAME OF THIS FUNCTION
def poseEstimation( R,
                    L,
                    x_prev,
                    y_prev,
                    theta_prev,
                    delta_phi_left,
                    delta_phi_right):
    """
        Calculate the current Duckiebot pose using dead reckoning approach,
        based on the kinematic model.

        Returns:
            x_curr, y_curr, theta_curr (:double: values)
    """
    x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
    y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
    theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/(L)
        
    #x_curr = y_curr = theta_curr=0
    
    return x_curr, y_curr, theta_curr

