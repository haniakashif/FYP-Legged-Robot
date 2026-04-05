import numpy as np

class THexConfig():

    mass_robot: float = 1.3984710000000002  # kg
    base_height_des: float = 0.02  # meters
    
    # inertia matrix of the whole robot from SolidWorks
    # NOTE: as the legs move, due to them not being negligible mass like for Cheetah 3, this would significantly change
    base_inertia = np.array([
        [0.02, 0.0, 0.0],
        [0.0, 0.01, 0.0],
        [0.0, 0.0, 0.03]
    ])
    
    # NOTE: the following parameters are somewhat hardcoded
    # choosing arbitrary value by considering moment arm to be straight and without intermediate angles (0.94140000000000001 * 3) / (0.02845 + 0.05439 + 0.02637 + 0.09265)
    fz_max = 14
    fz_min = 0.5

    swing_height = 0.02 # meters
    
    # PD Gains for the Swing Leg Controller
    kp_Cartesian = np.diag([80.0, 80.0, 80.0])
    kd_Cartesian = np.diag([10.0, 10.0, 10.0])
    
    # PD gains for Balance Controller
    Kp_balance_COM = np.diag([100.0, 100.0, 100.0])
    Kd_balance_COM = np.diag([5.0, 5.0, 5.0])
    Kp_balance_ori = np.diag([100.0, 100.0, 100.0])
    Kd_balance_ori = np.diag([5.0, 5.0, 5.0])
    
    S = np.diag([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
    alpha = 1e-4 * np.eye(12) 
    beta = 1e-3 * np.eye(12)