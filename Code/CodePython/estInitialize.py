import numpy as np
import scipy as sp
#NO OTHER IMPORTS ALLOWED (However, you're allowed to import e.g. scipy.linalg)

def estInitialize():
    # Fill in whatever initialization you'd like here. This function generates
    # the internal state of the estimator at time 0. You may do whatever you
    # like here, but you must return something that is in the format as may be
    # used by your estRun() function as the first returned variable.
    #
    # The second returned variable must be a list of student names.
    # 
    # The third return variable must be a string with the estimator type

    #we make the internal state a list, with the first three elements the position
    # x, y; the angle theta; and our favorite color. 
    
    # all given
    x_init = 0.0            
    y_init = 0.0            
    theta_init = np.pi/4
    B_init = 0.8             
    r_init = 0.425           


    P_init = np.diag([
        1.0**2, 
        1.0**2, 
        np.deg2rad(15.0)**2,
        (0.046)**2,
        (0.012)**2
    ])

    V = np.diag([0.01**2/4, 0.004**2/4])
    # V = np.diag([0.04**2,0.0001289 ])

    # W = np.diag([0.60**2, 0.60**2])
    W = np.diag([0.36,0.36 ])
  
    internalState = [x_init, y_init, theta_init,  B_init, r_init, P_init, V, W ]

    studentNames = ['Mingxuan Wang', 'Qiyuan Liu', 'Gechen Qu']
    
    estimatorType = 'EKF'  
    
    return internalState, studentNames, estimatorType

