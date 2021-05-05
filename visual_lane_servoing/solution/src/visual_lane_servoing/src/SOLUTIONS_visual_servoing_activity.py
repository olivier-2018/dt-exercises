#!/usr/bin/env python
# coding: utf-8

# In[5]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    
    steer_matrix_left_lane = np.zeros(shape)
    
    _, w = shape
    
    steer_matrix_left_lane[:, int(w/3):w] = -1.0

    return steer_matrix_left_lane

# In[5]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """
    
    steer_matrix_right_lane = np.zeros(shape)
    
    _, w = shape
    
    steer_matrix_right_lane[:, 0:int(2*w/3)] = 1.0

    return steer_matrix_right_lane

# In[21]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    # TODO: these are random values, you have to implement your own solution in here
    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Most of our operations will be performed on the grayscale version
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    sigma = 4

    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(img, (0,0), sigma)
    
    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, np.float32), angleInDegrees=False)
    
    threshold = 50

    mask_mag = (Gmag > threshold)
    
    white_lower_hsv = np.array([0/2, int(3*255/100), int(52*255/100)])
    white_upper_hsv = np.array([360/2, int(26*255/100), int(100*255/100)])
    yellow_lower_hsv = np.array([44/2, int(48*255/100), int(48*255/100)])
    yellow_upper_hsv = np.array([55/2, int(110*255/100), int(85*255/100)])

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)
    
    dilation_kernel = np.ones((5, 5), np.uint8)
    mask_yellow = cv2.dilate(mask_yellow, dilation_kernel, iterations=1)
    
    # Let's create masks for the left- and right-halves of the image
    width = img.shape[1]
    mask_left = np.ones(sobelx.shape, dtype=np.uint8)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0
    
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    mask_left_edge = mask_mag * mask_yellow
    mask_right_edge = mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    
    return (mask_left_edge, mask_right_edge)
