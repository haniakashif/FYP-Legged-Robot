import numpy as np

class RobotConfig:
    mass_robot: float
    base_height_des: float
    base_inertia: np.ndarray

    fz_max: float
    
    swing_height: float
    Kp_swing: np.ndarray
    Kd_swing: np.ndarray

class THexConfig(RobotConfig):
    # this is calculated from sdf by summing up mass
    # var = 0.30423+     0.14763+     0.03468+     0.09125+     0.14763+     0.03468+     0.09125+     0.14763+     0.034680999999999997+     0.09125+     0.14763+     0.03468+     0.09125
    
    # <ixx>0.00232</ixx>
    # <ixy>0</ixy>
    # <ixz>0</ixz>
    # <iyy>0.00064</iyy>
    # <iyz>0.00008</iyz>
    # <izz>0.00225</izz>
    
    mass_robot: float = 1.3984710000000002  # kg
    base_height_des: float = 0.10  # meters
    
    # symmetric intertial matirx from sdf
    # NOTE: this is inertia matrix for base only, not for whole robot
    base_inertia = np.array([
        [0.00232, 0.00, 0.00],
        [0.00, 0.00064, 0.00008],
        [0.00, 0.00008, 0.00225]
    ])
    
    # NOTE: i need whole body inertia matrix for balance controller, need to get that from solidworks

    # <effort> tag has torques for revolute joints, and force for prismatic joints. took 0.94140000000000001 from the sdf
    # 0.94140000000000001 * 3 is torque for one leg
    # Link Lengths
    # L1 = 2.845  # in cm
    # L2 = 5.439
    # L3 = 2.637
    # L4 = 9.265
    
    # NOTE: the following parameters are somewhat hardcoded
    # choosing arbitrary value by considering moment arm to be straight and without intermediate angles (0.94140000000000001 * 3) / (0.02845 + 0.05439 + 0.02637 + 0.09265)
    fz_max = 13.9908847716239
    fz_min = 1.0

    # How high the foot lifts during a step (meters)
    swing_height = 0.05
    
    # PD Gains for the Swing Leg Controller
    kp_Cartesian = np.diag([75.0, 75.0, 75.0])
    kd_Cartesian = np.diag([3.5, 3.5, 3.5])
    
    # PD gains for balance controller
    Kp_balance_COM = np.diag([100.0, 100.0, 100.0])
    Kd_balance_COM = np.diag([1.0, 1.0, 1.0])
    Kp_balance_ori = np.diag([100.0, 100.0, 100.0])
    Kd_balance_ori = np.diag([1.0, 1.0, 1.0])
    
    # NEED TO ADD BODY INERTIA MATRIX AS WELL AS S WEIGHT MATRIX
    S = np.diag([1.0, 1.0, 1.0, 10.0, 10.0, 10.0])
    alpha = 1e-4 * np.eye(12) 
    beta = 1e-3 * np.eye(12)