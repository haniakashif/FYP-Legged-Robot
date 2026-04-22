import numpy as np

class THexConfig():

    mass_robot: float = 1.3984710000000002  # kg
    base_height_des: float = 0.06 # meters
    
    # inertia matrix of the whole robot from SolidWorks
    # NOTE: as the legs move, due to them not being negligible mass like for Cheetah 3, this would significantly change
    base_inertia = np.array([
        [0.02, 0.0, 0.0],
        [0.0, 0.01, 0.0],
        [0.0, 0.0, 0.03]
    ])
    
    # choosing arbitrary value by considering moment arm to be straight and without intermediate angles (0.94140000000000001 * 3) / (0.02845 + 0.2439 + 0.02637 + 0.09265)
    fz_max = 14
    fz_min = 0.5

    swing_height = 0.04 # meters
    x_sprawl = 0.1
    y_sprawl = 0.1

    max_reach_cm = 17.3  # 0.04 cm safety margin
    
    # PD Gains for the Swing Leg Controller
    # kp_Cartesian = np.diag([2.0, 2.0, 2.0])
    # kd_Cartesian = np.diag([0.1, 0.1, 0.1])

    Kp_joint = np.diag([80.0, 80.0, 80.0])
    Kd_joint = np.diag([1.5, 1.5, 1.5])
    
    # PD gains for Balance Controller
    Kp_balance_COM = np.diag([60.0, 60.0, 60.0])
    Kd_balance_COM = np.diag([3.0, 3.0, 3.0])
    Kp_balance_ori = np.diag([100.0, 100.0, 100.0])
    Kd_balance_ori = np.diag([3.0, 3.0, 3.0])
    # Kp_balance_COM = np.diag([0.0, 0.0, 0.0])
    # Kd_balance_COM = np.diag([0.0, 0.0, 0.0])
    # Kp_balance_ori = np.diag([0.0, 0.0, 0.0])
    # Kd_balance_ori = np.diag([0.0, 0.0, 0.0])
    
    S = np.diag([1.0, 1.0, 1.0, 100.0, 100.0, 100.0])
    alpha = 1e-4 * np.eye(12) 
    beta = 1e-3 * np.eye(12)
    
    # Variances for stance (c) and swing (c_bar) transitions
    # These dictate how "steep" the erf curve is at touchdown and liftoff
    sigma_c0 = 0.2
    sigma_c1 = 0.2
    sigma_cbar0 = 0.2
    sigma_cbar1 = 0.2