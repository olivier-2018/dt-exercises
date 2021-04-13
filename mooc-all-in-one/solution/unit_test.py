import numpy as np

########### THIS IS JUST AN EXAMPLE ON HOW TO READ THE DATA.
# This data structure doesn't represent the real way the ROS messages are created,
# but it wants to give you an idea on how to access the data.
class stamp_struct: 
    secs=0
    nsecs=0

class encoder_struct:
    header= ''
    seq= 5594
    stamp= stamp_struct()
    frame_id= ""
    data= 13
    resolution=135
    type = 1
        
class UnitTestMessage:
            
    def __init__(self, callback):

        encoder_msg = encoder_struct()
        
        callback(encoder_msg)


class UnitTestOdometry:
    def __init__(self, R, baseline_wheel2wheel, poseEstimation):
        x_prev = y_prev = theta_prev = 0
        x_prev_ = []
        y_prev_ = []
        theta_prev_ = []
        
        for _ in range(0,35):
            x_prev_.append(x_prev) 
            y_prev_.append(y_prev)
            theta_prev_.append(theta_prev)
            x_prev, y_prev, theta_prev = poseEstimation( R,
                                            baseline_wheel2wheel,
                                            x_prev,
                                            y_prev,
                                            theta_prev,
                                            0.087266389,
                                            0.698131111)
        self.plot(x_prev_, y_prev_, theta_prev_)
        
    def plot(self,x,y,theta):
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots( 1 ) 
 
        axes.plot( x, y , 'r') 
        axes.set_aspect( 1 ) 
        
        plt.xlabel("X position")
        plt.ylabel("Y position")

        plt.title( 'Is it a circle?' ) 
        plt.show() 
    

class UnitTestPID:
    def __init__(self, PIDController, v_0):
        self.R = 0.0318
        self.L = 0.08
        self.theta_prev = 0
        self.PIDController = PIDController
        self.v_0 = v_0
        self.delta_t = 0.2
        self.t1 = np.arange(0.0, 10.0, self.delta_t)


    def test(self):
        omega = 0
        prev_e = 0
        prev_int = 0

        err_ = []
        theta_hat_ = []

        for _ in self.t1:
            theta_hat = self.sim(omega, self.v_0, self.delta_t)
            theta_hat_.append(theta_hat)
            err_.append(prev_e)
            u, prev_e, prev_int = self.PIDController(
                self.v_0, theta_hat, prev_e, prev_int, self.delta_t)

            self.v_0 = u[0]
            omega = u[1]

        self.plot(theta_hat_, err_)

    def plot(self, theta_hat_, err_):
        import matplotlib.pyplot as plt

        plt.axis([0, 10, np.min([np.min(theta_hat_),np.min(err_)]), np.max([np.max(theta_hat_),np.max(err_)])])
        plt.plot(self.t1, (theta_hat_), 'r--', self.t1, (err_), 'b')
        plt.legend(['Theta','error'])
        plt.show()



    def sim(self, omega, v, time):
        delta_phi_left = time*(v-omega*self.L)/self.R
        delta_phi_right = time*(v+omega*self.L)/self.R

        self.theta_prev = self.theta_prev + self.R * \
            (delta_phi_right-delta_phi_left)/(2*self.L)
        return self.theta_prev
