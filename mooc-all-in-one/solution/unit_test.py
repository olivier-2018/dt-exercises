import numpy as np
        
class UnitTestMessage:
    def __init__(self, callback):
        from duckietown_msgs.msg import WheelEncoderStamped 
        from std_msgs.msg import Header
        import time
        
        
        header = Header()
        header.frame_id = f"agent/left_wheel_axis"
        header.stamp.secs = 0  # rospy.Time.now() is the correct stamp, anyway this works only when a node is initialized
        header.stamp.nsecs = 0
        
        encoder_msg=WheelEncoderStamped(
            header=header,
            data=13,
            resolution=135,
            type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
        )
        
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
    def __init__(self, R, baseline, v_0, gain, trim, PIDController):
        self.R = R
        self.L = baseline
        self.PIDController = PIDController
        self.delta_t = 0.2
        self.t1 = np.arange(0.0, 10.0, self.delta_t)
        self.theta_prev = 0
        self.v_0 = v_0
        
        self.k_r_inv = (gain + trim) / 27.0
        self.k_l_inv = (gain - trim) / 27.0


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
        
        self.plot(theta_hat_, err_,"No noise on theta","Time (s)","Theta (Degree)")
        
        self.theta_prev = 0
        omega = 0
        prev_e = 0
        prev_int = 0
        
        err_ = []
        theta_hat_ = []
        
        for _ in self.t1:
            theta_hat = self.sim_noise(omega, self.v_0, self.delta_t)
            theta_hat_.append(theta_hat)
            err_.append(prev_e)
            u, prev_e, prev_int = self.PIDController(
                self.v_0, theta_hat, prev_e, prev_int, self.delta_t)

            self.v_0 = u[0]
            omega = u[1]

        self.plot(theta_hat_, err_,"Theta with noise","Time (s)","Theta (Degree)")

    def plot(self, theta_hat_, err_, title, x_label, y_label):
        import matplotlib.pyplot as plt
        
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        theta_hat_deg=[]
        for el in theta_hat_:
            theta_hat_deg.append(el*180/np.pi)
        err_deg=[]
        for el in err_:
            err_deg.append(el*180/np.pi)
            
        plt.axis([0, 10, np.min([np.min(theta_hat_deg),np.min(err_deg)]), np.max([np.max(theta_hat_deg),np.max(err_deg)])])
        plt.plot(self.t1, (theta_hat_deg), 'r--', self.t1, (err_deg), 'b')
        plt.legend(['Theta','error'])
        plt.show()



    def sim(self, omega, v, time):
        delta_phi_left = time*(v-omega*self.L)/self.R
        delta_phi_right = time*(v+omega*self.L)/self.R

        self.theta_prev = self.theta_prev + self.R * \
            (delta_phi_right-delta_phi_left)/(2*self.L)
        return self.theta_prev
    
    def sim_noise(self, omega, v, time):
        delta_phi_left = time*(v-omega*self.L)/self.R
        delta_phi_right = time*(v+omega*self.L)/self.R

        self.theta_prev = self.theta_prev + self.R * \
            (delta_phi_right-delta_phi_left)/(2*self.L)
        return self.theta_prev+np.random.normal(0, 0.017453278) # 1 degree of variance
    
    def wheel_inputs(self):
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv
