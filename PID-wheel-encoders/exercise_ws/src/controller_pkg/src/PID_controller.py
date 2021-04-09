#!/usr/bin/env python
# coding: utf-8

# In[3]:


import numpy as np
# Heading control
def PIDController(v_0, theta_hat, prev_e, prev_int, delta_t):
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
    ## Set outside this function:
    # gain = 0.45
    # x0, y0, theta0 = [0, 0, 0]
    
    
    # theta_ref = 30*pi/180 # has to be different from 0, because 0,0,0 are assumed as initial conditions
    
    # Dtheta = ((delta_phi_right - delta_phi_left)/2*L)*R # insert correct Dtheta function
    
    # theta_hat = theta_hat + Dtheta
    
    # e = theta_ref - theta_hat 
    
    # e_int = prev_e + e*delta_t

    # e_der = (e - prev_e)/delta_t
    v_0=0.15

    theta_ref = -70*np.pi/180

    # error
    e = theta_ref - theta_hat

    # integral of the error
    e_int = prev_int + e*delta_t

    # antiwindup
    e_int = max(min(e_int,2),-2)


    # derivative of the error
    e_der = (e - prev_e)/delta_t

    Kp=2
    Ki=0.001
    Kd=0.15
    
    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der
    
    u = [v_0, omega]
    
    print(f"\n\nDelta time : {delta_t} \nE : {e} \nE int : {e_int} \nPrev e : {prev_e}\nU : {u} \nTheta : {theta_hat} \n")

    
    return u, e, e_int


## Exercise: lateral position control 

# v_0 = u[0]
# rosparam set /!hostname/kinematics_node/gain v_0


# for the exercise: no longer heading control, but y direction control

# challenge used to evaluate is LF
# world is a straight lane (or whatever)
# initial condition of the robot is in the opposite lane
# (we are playing now with (d, theta))
# d0 = -2 (whatever brings you in the opposite lane)
# theta0 = whatever = 0
# d_ref = 0 # (this means be at the center of the lane, as in regular lane following)
# e = d_ref - dhat
# (dhat is initialized with d0)
# dhat = dlast + delta_d
# theta = R/(2*L)*(delta_phi_right-delta_phi_left)
# delta_d = R/2*(delta_phi_l + delta_phi_r)*cos(theta) #(double check cos)
# v = gain = 0.5 (whatever)
# omega = PID(e)
# u = [v omega]



# To get the exrecise done, what do students need to know? 
# ACTIVITY 1. Assume that if v_l = v_r, the robot goes straight. Why? because this is what the model we are using tells us. 
# In reality, this is not the case -> wheel calibration, i.e., find trim so that the robot goes kind of straight

# ACTIVITY 2. We need R, L --> odometry calibration --> Nice to have least square approach, but manual measurement works fine as well

# ACTIVITY 3. Deadreckoning model --> odometry-activity 

# ACTIVITY 4. Understand how PID works.. --> heading control

# Exercise: use all the above together to do lateral position control. 

