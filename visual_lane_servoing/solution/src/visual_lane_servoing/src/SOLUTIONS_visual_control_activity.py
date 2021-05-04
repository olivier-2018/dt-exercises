#!/usr/bin/env python
# coding: utf-8

# In[5]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np

#TODO: write a correct function

def LMOrientation(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            theta_left:  Image-space orientation (radians) of the left (dashed-yellow) lane marking (double)
            theta_right: Image-space orientation (radians) of the right (solid-white) lane marking (double)
            mask_left:   Masked image for the dashed-yellow line (BGR, just for visualization)
            mask_right:   Masked image for the solid-white line (BGR, just for visualization)
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
    
#     white_lower_hsv = np.array([0/2, 3*255/100, 52*255/100])
#     white_upper_hsv = np.array([360/2, 26*255/100, 96*255/100])
#     yellow_lower_hsv = np.array([44/2, 48*255/100, 48*255/100])
#     yellow_upper_hsv = np.array([55/2, 110*255/100, 85*255/100])
    
    
    
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
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0
    
    
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    # Let's generate the complete set of masks, including those based on color
#     mask_left_edge = mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
#     mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    
    
    mask_left_edge = mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    
    
    left_masked_img = cv2.addWeighted(img_gaussian_filter, 0.4, (Gmag * mask_left_edge).astype(np.uint8), 0.6, 0)
    right_masked_img = cv2.addWeighted(img_gaussian_filter, 0.4, (Gmag * mask_right_edge).astype(np.uint8), 0.6, 0)
    
    
    
#     ax8.imshow(img_gaussian_filter,cmap = 'gray')
#     ax8.imshow(Gmag * mask_right_edge, cmap='jet', alpha=0.5)
    
    # Determine the orientation of the left and right gradients as the mode of each histogram
    (hist_left_edge, bins_left_edge) = np.histogram(np.extract(mask_left_edge, Gdir).flatten(), bins=30)
    (hist_right_edge, bins_right_edge) = np.histogram(np.extract(mask_right_edge, Gdir).flatten(), bins=30)

    idx = hist_left_edge.argmax()
    theta_left = (bins_left_edge[idx] + bins_left_edge[idx+1])/2
    idx = hist_right_edge.argmax()
    theta_right = (bins_right_edge[idx] + bins_right_edge[idx+1])/2

    return theta_left, theta_right, left_masked_img, right_masked_img


# In[12]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

#TODO: write a correct function

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
    theta_left_desired = 3.84 # (220 degrees)
    theta_right_desired = 5.59 # (320 degrees)
    
    # TODO: These are arbitrary values. You need to implement your own solution here
    residual_left = 0.0
    residual_right = 0.0


    return residual_left, residual_right

