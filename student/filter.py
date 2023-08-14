# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        # system matrix
        dt = params.dt

        F = np.matrix([[1, 0, 0, dt, 0, 0],
                       [0, 1, 0, 0, dt, 0],
                       [0, 0, 1, 0, 0, dt],
                       [0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 0, 1]])

        return F

    def Q(self):
        # process noise covariance Q
        q = params.q
        dt = params.dt
        q1 = ((dt**3)/3) * q
        q2 = ((dt**2)/2) * q
        q3 = dt * q

        Q = np.matrix([[q3, 0, 0, q2, 0, 0],
                       [0, q3, 0, 0, q2, 0],
                       [0, 0, q3, 0, 0, q2],
                       [q2, 0, 0, q1, 0, 0],
                       [0, q2, 0, 0, q1, 0],
                       [0, 0, q2, 0, 0, q1]])

        return Q

    def predict(self, track):
        # predict state and estimation error covariance to next timestep
        x = self.F() * track.x  # state prediction
        P = self.F() * track.P * self.F().transpose() + self.Q()  # covariance prediction
        track.set_x(x)
        track.set_P(P)

    def update(self, track, meas):
        # update state and covariance with associated measurement
        H = meas.sensor.get_H(track.x)  # measurement matrix
        S = self.S(track, meas, H)  # covariance of residual
        K = track.P * H.transpose() * np.linalg.inv(S)  # Kalman gain
        x = track.x + K * self.gamma(track, meas)  # state update
        I = np.identity(params.dim_state)
        P = (I - K*H) * track.P  # covariance update

        track.set_x(x)
        track.set_P(P)
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        # residual
        gamma = meas.z - meas.sensor.get_hx(track.x)

        return gamma

    def S(self, track, meas, H):
        # covariance of residual
        S = H * track.P * H.transpose() + meas.R

        return S
