import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estRun(time, dt, internalStateIn, steeringAngle, pedalSpeed, measurement):
    # In this function you implement your estimator. The function arguments
    # are:
    #  time: current time in [s]
    #  dt: current time step [s]
    #  internalStateIn: the estimator internal state, definition up to you. 
    #  steeringAngle: the steering angle of the bike, gamma, [rad] 
    #  pedalSpeed: the rotational speed of the pedal, omega, [rad/s] 
    #  measurement: the position measurement valid at the current time step
    #
    # Note: the measurement is a 2D vector, of x-y position measurement.
    #  The measurement sensor may fail to return data, in which case the
    #  measurement is given as NaN (not a number).
    #
    # The function has four outputs:
    #  x: your current best estimate for the bicycle's x-position
    #  y: your current best estimate for the bicycle's y-position
    #  theta: your current best estimate for the bicycle's rotation theta
    #  internalState: the estimator's internal state, in a format that can be understood by the next call to this function

    # Example code only, you'll want to heavily modify this.
    # this internal state needs to correspond to your init function:
    
    x, y, theta, B, r, P, V, W = internalStateIn

    vel = r * 5.0 * pedalSpeed 

    # state prediction
    x_pred = np.array([
        x + vel * dt * np.cos(theta),                       # 0:= x_p
        y + vel * dt * np.sin(theta),                       # 1:= y_p
        theta + (vel * dt / B) * np.tan(steeringAngle),     # 2:= theta_p
        B,                                                  # 3:= B_p
        r                                                   # 4:= r_p
    ])

    # EKF linearized A matrix
    A = np.eye(5)
    A[0,2] = -vel * dt * np.sin(theta)
    A[0,4] =  5.0 * pedalSpeed * dt * np.cos(theta)
    A[1,2] =  vel * dt * np.cos(theta)
    A[1,4] =  5.0 * pedalSpeed * dt * np.sin(theta)
    A[2,3] = -vel * dt * np.tan(steeringAngle) / (B**2)
    A[2,4] = (5.0 * pedalSpeed * dt / B) * np.tan(steeringAngle)

    # EKF linearized L matrix
    L = np.zeros((5,2))
    L[3, 0] = 1.0
    L[4, 1] = 1.0

    # L[0, 1] = 5 * pedalSpeed * np.cos(theta)
    # L[0, 1] = 5 * pedalSpeed * np.cos(theta)
    # variance prediction
    P_pred = A @ P @ A.T + L @ V @ L.T ######################################################

    if not np.any(np.isnan(measurement)):

        # measurement update
        z_pred = np.array([
            x_pred[0] + 0.5 * x_pred[3] * np.cos(x_pred[2]),
            x_pred[1] + 0.5 * x_pred[3] * np.sin(x_pred[2]) 
        ])

        # EKF linearized H matrix
        H = np.zeros((2,5))
        H[0,0] =  1.0
        H[0,2] = -0.5 * x_pred[3] * np.sin(x_pred[2])
        H[0,3] =  0.5 * np.cos(x_pred[2])
        H[1,1] =  1.0
        H[1,2] =  0.5 * x_pred[3] * np.cos(x_pred[2])
        H[1,3] =  0.5 * np.sin(x_pred[2])

        # EKF linearized M matrix
        M = np.eye(2)

        # EKF gain
        K  = np.array(P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + M @ W @ M.T))

        # update
        x_meas = x_pred + K @ (measurement - z_pred)
        P_meas = np.array(np.eye(5) - K @ H) @ P_pred @ np.array(np.eye(5) - K @ H).T + K @ M @ W @ M.T @ K.T
    
    else:

        x_meas = x_pred
        P_meas = P_pred
    
    # x, y, theta, B, r, P, V, W
    internalStateOut = [x_meas[0], x_meas[1], x_meas[2], x_meas[3], x_meas[4], P_meas, V, W]

    # DO NOT MODIFY THE OUTPUT FORMAT:
    return x_meas[0], x_meas[1], x_meas[2], internalStateOut 


