#!/usr/bin/env python
# coding: utf-8

# In[5]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your PIDController function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import numpy as np

# TODO: write your PID function for heading control!

def PIDController(v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t):
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        theta_ref (:double:) reference heading pose
        theta_hat (:double:) the current estiamted theta.
        prev_e (:double:) tracking error at previous iteration.
        prev_int (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
    # TODO: these are random values, you have to implement your own PID controller in here
    omega = np.random.uniform(-8.0, 8.0)
    e = np.random.random()
    e_int = np.random.random()
    
    # SOLUTION
    # Tracking error
    e = theta_ref - theta_hat

    # integral of the error
    e_int = prev_int + e*delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2),-2)

    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    Kp = 5
    Ki = 0.2
    Kd = 0.1

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der
    
    #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
        
    return [v_0, omega], e, e_int


# In[3]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import numpy as np
import cv2

#TODO: write a correct function

def estimate_lane_relative_heading(H, image):
    """
        Args:
            H: The 3x3 image-to-ground plane homography (numpy.ndarray)
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            theta_hat: An estimate of the robot's heading (radians) relative to the lane (float)
            lines_left:  An n x 4 array of candidate lines for the right lane markings (numpy.ndarray)
                         Each row [x0, y0, x1, y1] specifies the line-segment coordinate
            lines_right: An m x 4 array of candidate lines for the left lane markings (numpy.ndarray)
                         Each row [x0, y0, x1, y1] specifies the line-segment coordinate
    """
    
    # TODO: these are random values, you have to implement your own solution in here
    theta_hat = np.random.random()
    lines_left = np.array([])
    lines_right = np.array([])
    
    # SOLUTION
    
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    Hinv = np.linalg.inv(H)
    
    # Masking the horizon
    xmax = 2
    X = np.array([xmax, 0.0, 1.0])
    x = Hinv.dot(X)
    x = x/x[-1]

    height, width, _ = image.shape
    mask_ground = np.ones((height, width), dtype=np.uint8)
    mask_ground[0:int(np.floor(x[1])),:] = 0
    
    
    # Gaussian smoothing
    sigma = 2
    img_gaussian_filter = cv2.GaussianBlur(image,(0,0), sigma)

    # Canny edge detection
    canny_lower_threshold = 80
    canny_upper_threshold = 200
    canny_kernel_size = 3;

    edges = cv2.Canny(img_gaussian_filter, canny_lower_threshold, canny_upper_threshold, canny_kernel_size)

    
    # Color-based masking
    # Using the above tool, we can identify the bounds as follows
    # Since OpenCV uses a Hue range of [0, 179], we need to divide the Hue values above by 2
    white_lower_hsv = np.array([0/2, 3*255/100, 52*255/100])
    white_upper_hsv = np.array([360/2, 26*255/100, 96*255/100])
    yellow_lower_hsv = np.array([44/2, 47*255/100, 48*255/100])
    yellow_upper_hsv = np.array([55/2, 100*255/100, 85*255/100])

    mask_white_orig = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow_orig = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    # Color dilation
    dilation_kernel = np.ones((11, 11), np.uint8)

    mask_yellow = cv2.dilate(mask_yellow_orig, dilation_kernel, iterations=1)
    mask_white = cv2.dilate(mask_white_orig, dilation_kernel, iterations=1)
    
    # Masking
    mask_left_edge = mask_ground * mask_yellow
    mask_right_edge = mask_ground * mask_white
  
    # Hough Transform
    # Parameters for left line detection
    hough_threshold_left = 50
    hough_min_line_length_left = 10
    hough_max_line_gap_left = 50
    hough_rho_left = 1
    hough_theta_left = 1*np.pi/180

    # Parameters for white line detection
    hough_threshold_right = 50 #100
    hough_min_line_length_right = 2 #10
    hough_max_line_gap_right = 15
    hough_rho_right = 1
    hough_theta_right = 1 * np.pi/180

    lines_left = cv2.HoughLinesP(edges * mask_left_edge,
                            rho=hough_rho_left,
                            theta=hough_theta_left,
                            threshold=hough_threshold_left,
                            minLineLength=hough_min_line_length_left,
                            maxLineGap=hough_max_line_gap_left)




    lines_right = cv2.HoughLinesP(edges * mask_right_edge,
                            rho=hough_rho_right,
                            theta=hough_theta_right,
                            threshold=hough_threshold_right,
                            minLineLength=hough_min_line_length_right,
                            maxLineGap=hough_max_line_gap_right)


    (thetas_left, thetas_right) = get_lane_marking_orientations(H, lines_left, lines_right)
    thetas = np.append(thetas_left, thetas_right)

    # Determine the orientation of the left and right gradients as the mode of each histogram
    (hist_thetas, bins_thetas) = np.histogram(thetas, bins=30)

    idx = hist_thetas.argmax()
    theta_argmax = (bins_thetas[idx] + bins_thetas[idx+1])/2
    theta_mean = np.mean(thetas)

    theta_hat = theta_argmax

    return (theta_hat, lines_left, lines_right)



def project_image_to_ground(H, x):
    """
        Args:
            H: The 3x3 image-to-ground plane homography (numpy.ndarray)
            x: An array of non-homogeneous image coordinates, one per column (numpy.ndarray)
        Returns:
            X: An array of non-homogeneous coordinates in the world (ground) frame, one per column (numpy.ndarray)
    """
    
    if x.shape[0] == 2:
        if x.ndim == 1:
            x = np.append(x, 1)
        else:
            x = np.vstack((x, np.ones((1, x.shape[1]))))
    
    X = H.dot(x)
    X = X/X[2,None]
    
    return X[0:2,]



def get_lane_marking_orientations(H, lines_left, lines_right):
    """
        Computes the orientation of any detected lines in the ground frame

        Args:
            H:           The 3x3 image-to-ground plane homography (numpy.ndarray)
            lines_left:  An n x 4 array of candidate lines for the right lane markings
                         Each row [x0, y0, x1, y1] specifies the line-segment coordinate
            lines_right: An m x 4 array of candidate lines for the left lane markings
                         Each row [x0, y0, x1, y1] specifies the line-segment coordinate
                         
        Returns:
            theta_left:  An n-element array of the orientations of each candidate line 
                         for the left lane markings
            theta_right  An n-element array of the orientations of each candidate line 
                         for the right lane markings
    """
    
    thetas_left = np.array([])
    thetas_right = np.array([])
    if lines_left is not None:
        for line in lines_left:
            [[x1, y1, x2, y2]] = line
            xy1 = np.array([[x1, y1]]).transpose()
            xy2 = np.array([[x2, y2]]).transpose()

            # Project to the ground frame
            XY1 = project_image_to_ground(H, xy1)
            XY2 = project_image_to_ground(H, xy2)

            X = np.array([XY1[0], XY2[0]])
            Y = np.array([XY1[1], XY2[1]])
            
            ind = np.argsort(X, axis=0)
            X = np.take_along_axis(X, ind, axis=0)
            Y = np.take_along_axis(Y, ind, axis=0)
            
            theta = np.arctan2(Y[1]-Y[0], X[1]-X[0])
            thetas_left = np.append(thetas_left, theta)

    if lines_right is not None:
        # Visualize the edges projeted on to the ground plane
        for line in lines_right:
            [[x1, y1, x2, y2]] = line
            xy1 = np.array([[x1, y1]]).transpose()
            xy2 = np.array([[x2, y2]]).transpose()

            # Project to the ground frame
            XY1 = project_image_to_ground(H, xy1)
            XY2 = project_image_to_ground(H, xy2)

            X = np.array([XY1[0], XY2[0]])
            Y = np.array([XY1[1], XY2[1]])
            
            ind = np.argsort(X, axis=0)
            X = np.take_along_axis(X, ind, axis=0)
            Y = np.take_along_axis(Y, ind, axis=0)

            theta = np.arctan2(Y[1]-Y[0], X[1]-X[0])
            thetas_right = np.append(thetas_right, theta)
            
    return (thetas_left, thetas_right)

