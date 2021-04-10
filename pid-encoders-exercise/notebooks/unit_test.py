import numpy as np


class UnitTestPID:
    def __init__(self, PIDController, v_0):
        self.R = 0.0318
        self.L = 0.08
        self.theta_prev = 0
        self.x_prev = 0
        self.PIDController = PIDController
        self.v_0 = v_0
        self.delta_t = 0.2
        self.t1 = np.arange(0.0, 30.0, self.delta_t)

    def test(self):
        omega = 0
        
        prev_e_x = 0
        prev_int_x = 0
        
        err_x_ = []
        
        x_hat_ = []

        for _ in self.t1:
            x_hat = self.sim(omega, self.v_0, self.delta_t)
            
            x_hat_.append(x_hat)
            
            u, prev_e_x, prev_int_x = self.PIDController(
                self.v_0, x_hat, prev_e_x, prev_int_x, self.delta_t)
            
            err_x_.append(prev_e_x)

            self.v_0 = u[0]
            omega = u[1]

        self.plot(x_hat_, err_x_)

    def plot(self, x_hat_, err_x_):
        import matplotlib.pyplot as plt
        
        print(f"err_x {err_x_[-1]}, x_hat {x_hat_[-1]}")

        plt.axis([0, 30, np.min([np.min(x_hat_), np.min(err_x_)]), 
            np.max([np.max(x_hat_), np.max(err_x_)])])
        plt.plot(self.t1, (x_hat_), 'r--', self.t1, (err_x_), 'c')
        plt.legend(['x', 'x error'])
        plt.show()

    def sim(self, omega, v, time):
        delta_phi_left = time*(v-omega*self.L)/self.R
        delta_phi_right = time*(v+omega*self.L)/self.R
        
        self.x_prev = self.x_prev + self.R * \
            (delta_phi_right+delta_phi_left)*np.cos(self.theta_prev)/2

        self.theta_prev = self.theta_prev + self.R * \
            (delta_phi_right-delta_phi_left)/(2*self.L)

        return self.x_prev
