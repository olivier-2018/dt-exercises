import numpy as np


class UnitTestPID:
    def __init__(self, PIDController, v_0):
        self.R = 0.0318
        self.L = 0.08
        self.theta_prev = 0
        self.y_prev = 0
        self.PIDController = PIDController
        self.v_0 = v_0
        self.delta_t = 0.2
        self.t1 = np.arange(0.0, 30.0, self.delta_t)

    def test(self):
        omega = 0
        
        prev_e_y = 0
        prev_int_y = 0
        
        err_y_ = []
        
        y_hat_ = []

        for _ in self.t1:
            y_hat = self.sim(omega, self.v_0, self.delta_t)
            
            y_hat_.append(y_hat)
            
            u, prev_e_y, prev_int_y = self.PIDController(
                self.v_0, y_hat, prev_e_y, prev_int_y, self.delta_t)
            
            err_y_.append(prev_e_y)

            self.v_0 = u[0]
            omega = u[1]

        self.plot(y_hat_, err_y_)

    def plot(self, y_hat_, err_y_):
        import matplotlib.pyplot as plt
        
        print(f"err_y {err_y_[-1]}, y_hat {y_hat_[-1]}")

        plt.axis([0, 30, np.min([np.min(y_hat_), np.min(err_y_)]), 
            np.max([np.max(y_hat_), np.max(err_y_)])])
        plt.plot(self.t1, (y_hat_), 'r--', self.t1, (err_y_), 'c')
        plt.legend(['Y', 'Y error'])
        plt.show()

    def sim(self, omega, v, time):
        delta_phi_left = time*(v-omega*self.L)/self.R
        delta_phi_right = time*(v+omega*self.L)/self.R
        
        self.y_prev = self.y_prev + self.R * \
            (delta_phi_right+delta_phi_left)*np.sin(self.theta_prev)/2

        self.theta_prev = self.theta_prev + self.R * \
            (delta_phi_right-delta_phi_left)/(2*self.L)

        return self.y_prev
