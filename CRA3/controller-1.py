import math

class Controller():

    def __init__(self):

        # EX 1:  Gains for controller
        self.k_P = 0
        self.k_I = 0

        # EX 1: initialize variable for integral
        self.C_I = 0

        # EX 2:Specify the sampling time. k_s = 2 means that the time between two
        # executions gets doubled (for k_s= 1: T =~ 85ms)
        self.k_s = 1

        # EX 3: Specify the time delay. k_d = 1 means that there is a time delay
        # t_delay = t_delay(0) + k_d * T where T = 70ms
        # delay is added on top of the slower sampling rate if both are present
        self.k_d = 0

        # EX 4: Saturation of motors [rad/s]
        self.u_sat = 2

        # EX 4: Feedback gain for anti-windup
        self.k_t = 0

    # Inputs:   d_est   Estimation of distance from lane center (positve when
    #                   offset to the left of driving direction) [m]
    #           phi_est Estimation of angle of bot (positive when angle to the
    #                   left of driving direction) [rad]
    #           d_ref   Reference of d (for lane following, d_ref = 0) [m]
    #           v_ref   Reference of velocity [m/s]
    #           t_delay Delay it took from taking image up to now [s]
    #           dt_last Time it took from last processing to current [s]

    # Output:   v_out       velocity of Duckiebot [gain, element of [0,1]]
    #           omega_out   angular velocity of Duckiebot [rad/s]

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):

        #TODO


        return (v_out, omega_out)
