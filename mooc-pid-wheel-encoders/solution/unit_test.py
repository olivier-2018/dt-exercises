import numpy as np


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
