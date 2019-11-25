import math
import numpy as np
import control as ctl


class Controller():

    def __init__(self):

        # Define cost matrices here:
        # define numpy matrices with: self.A = numpy.array([[a11, a12], [a21, a22]])

        #TODO: add discretized matrices

        # Inputs:   d_est   Estimation of distance from lane center (positve when
        #                   offset to the left of driving direction) [m]
        #           phi_est Estimation of angle of bot (positive when angle to the
        #                   left of driving direction) [rad]
        #           d_ref   Reference of d (for lane following, d_ref = 0) [m]
        #           v_ref   Reference of velocity [m/s]
        #           t_delay Delay it took from taking image up to now [s]
        #           dt_last Time it took from last processing to current [s]

        # Output:   v_out       velocity of Duckiebot [m/s]
        #           omega_out   angular velocity of Duckiebot [rad/s

        return

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):
        # Exception handler for dt_last==0
        if (dt_last == 0): dt_last = 0.0001
        # Your code goes here:

        #TODO implement LQR

        return (v_out, omega_out)

