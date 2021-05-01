#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write a correct function

import numpy as np
import cv2

def LMOrientation(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            theta_left:  Image-space orientation (radians) of the left (dashed-yellow) lane marking (double)
            theta_right: Image-space orientation (radians) of the right (solid-white) lane marking (double)
    """

    # TODO: these are random values, you have to implement your own solution in here

    theta_left = 3.84  # (220 degrees)
    theta_right = 5.59  # (320 degrees)

    return theta_left, theta_right


# In[ ]:


# The function written in this cell will actually be ran on your robot (sim or real).
# Put together the steps above and write your DeltaPhi function!
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write a correct function

import numpy as np


def getMotorResiduals(theta_left, theta_right):
    """
        Args:
            theta_left:  Image-space orientation (radians) of the left (dashed-yellow) lane marking (double)
            theta_right: Image-space orientation (radians) of the right (solid-white) lane marking (double)
        Return:
            residual_left:  Residual command for the left motor (double)
            residual_right: Residual command for the right motor (double)
    """

    # TODO: These represent the desired orientations of the left and right lane markings
    #       You should change these values to represent the orientations associated with
    #       the robot positioned in the middle of the lane in the direction of travel.
    theta_left_desired = 3.84  # (220 degrees)
    theta_right_desired = 5.59  # (320 degrees)

    # TODO: These are arbitrary values. You need to implement your own solution here
    residual_left = 0.0
    residual_right = 0.0

    return residual_left, residual_right

