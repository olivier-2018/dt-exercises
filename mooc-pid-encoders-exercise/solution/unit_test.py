import numpy as np


    
class UnitTestPositionPID:
    def __init__(self, R, baseline, gain, trim, PIDController):
        self.R = R
        self.L = baseline
        self.PIDController = PIDController
        self.delta_t = 0.2
        self.test_horizont = 50.0
        self.t1 = np.arange(0.0, self.test_horizont, self.delta_t)
        self.theta_prev = 0
        self.y_prev = 0
        self.y_ref=-0.10
        self.v_0 = 0.2
        
        self.k_r_inv = (gain + trim) / 27.0
        self.k_l_inv = (gain - trim) / 27.0
        


    def test(self):
        omega = 0
        prev_e = 0
        prev_int = 0

        err_ = []
        y_hat_ = []
        u_r_ = []
        u_l_ = []
        
        for _ in self.t1:
            y_hat,u_r,u_l = self.sim(omega, self.v_0, self.delta_t)
            
            y_hat_.append(y_hat)
            err_.append(prev_e)
            u_r_.append(u_r)
            u_l_.append(u_l)
        
            u, prev_e, prev_int = self.PIDController(
                self.v_0,self.y_ref, y_hat, prev_e, prev_int, self.delta_t)

            self.v_0 = u[0]
            omega = u[1]
        
        self.plot_pose(y_hat_, err_,"No noise","Time (s)","Y (m)")
        self.plot_input(u_r_,u_l_,"Control inputs","Time (s)","PWM?")
        
        self.theta_prev = 0
        self.y_prev = 0
        omega = 0
        prev_e = 0
        prev_int = 0
        
        err_ = []
        y_hat_ = []
        u_r_ = []
        u_l_ = []
        
        for _ in self.t1:
            y_hat,u_r,u_l = self.sim_noise(omega, self.v_0, self.delta_t)
            
            y_hat_.append(y_hat)
            err_.append(prev_e)
            u_r_.append(u_r)
            u_l_.append(u_l)
        
            u, prev_e, prev_int = self.PIDController(
                self.v_0,self.y_ref, y_hat, prev_e, prev_int, self.delta_t)

            self.v_0 = u[0]
            omega = u[1]

        self.plot_pose(y_hat_, err_,"With noise","Time steps (0.2 s)","Y (m)")
        self.plot_input(u_r_,u_l_,"Control inputs","Time steps (0.2 s)","PWM?")

    def plot_input(self, u_r, u_l, title, x_label, y_label):
        import matplotlib.pyplot as plt
        
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.xticks(np.arange(0, len(u_l)+1, 1))
        
        # plot the control inputs
        plt.axis([0, self.test_horizont, np.min([np.min(u_r),np.min(u_l)]), np.max([np.max(u_r),np.max(u_l)])])
        
        plt.plot(self.t1, (u_r), 'r--', self.t1, (u_l), 'b')
        
        plt.legend(['Right wheel','Left wheel'])
        plt.show()
        
    def plot_pose(self, y_hat_, err_, title, x_label, y_label):
        import matplotlib.pyplot as plt
        
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.xticks(np.arange(0, len(y_hat_)+1, 1))
        
        # plot the error and position
        plt.axis([0, self.test_horizont, np.min([np.min(y_hat_),np.min(err_)]), np.max([np.max(y_hat_),np.max(err_)])])
        plt.plot(self.t1, (y_hat_), 'r--', self.t1, (err_), 'b')
        
        plt.legend(['Y','error'])
        plt.show()



    def sim(self, omega, v, time):
        omega_l = (v-0.5*omega*self.L)/self.R
        omega_r = (v+0.5*omega*self.L)/self.R
        
        delta_phi_left = time*omega_l
        delta_phi_right = time*omega_r
        
        self.y_prev = self.y_prev + self.R * \
            (delta_phi_right+delta_phi_left)*np.sin(self.theta_prev)/2

        self.theta_prev = self.theta_prev + self.R * \
            (delta_phi_right-delta_phi_left)/(self.L)
        
        u_r,u_l=self.wheel_inputs(omega_r, omega_l)
        
        return self.y_prev,u_r,u_l
    
    def sim_noise(self, omega, v, time):
        omega_l = (v-0.5*omega*self.L)/self.R
        omega_r = (v+0.5*omega*self.L)/self.R
        
        delta_phi_left = time*omega_l
        delta_phi_right = time*omega_r

        self.y_prev = self.y_prev + self.R * \
            (delta_phi_right+delta_phi_left)*np.sin(self.theta_prev)/2 + np.random.normal(0, 0.005) # 1 cm variance
        
        self.theta_prev = self.theta_prev + self.R * \
            (delta_phi_right-delta_phi_left)/(2*self.L) # + np.random.normal(0, 0.017453278) # 1 degree of variance
        
        u_r,u_l=self.wheel_inputs(omega_r, omega_l)
        
        return self.y_prev,u_r,u_l 
    
    def wheel_inputs(self, omega_r, omega_l):
        u_r = omega_r * self.k_r_inv
        u_l = omega_l * self.k_l_inv
        
        u_r = np.max([np.min([u_r,1]),-1])
        u_l = np.max([np.min([u_l,1]),-1])
        
        return u_r,u_l
