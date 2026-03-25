import numpy as np

# base parameters for controller
class LinearMpcConfig:
    # main clock running at 1/dt_control hz
    dt_control: float = 0.002
    
    # mpc is running at 1000/20 = 50Hz, which means mpc is being evaluated every 20 iterations of the main clock
    iteration_between_mpc: int = 1
    dt_mpc: float = dt_control * iteration_between_mpc

    horizon: int = 64      # mpc optimization horizon

    gravity: np.float32 = 9.81
    friction_coef: float = 0.7

    # QP weights from the minimization equation 
    # state: [roll, pitch, yaw, x, y, z, wx, wy, wz, vx, vy, vz, gravity]
    # control input: [fx_FL, fy_FL, fz_FL, fx_FR, fy_FR, fz_FR, fx_RL, fy_RL, fz_RL, fx_RR, fy_RR, fz_RR] forces for all 4 feet
    
    # the gravity is added in Q to let Ax + Bu remain as a linear optimization problem, else there would have been a + gravity at the end. To accommodate gravity, we have set it to 0, i.e. derivative of gravity is 0 and we're not supposed to minimize it or anything, simply let it exist in the optimization
    
    # the control inputs are all very small to ensure that whatever foot forces are being used, they get scaled down so they dont approach infinity, and it also helps to minimize them
    
    # weight matrices used in MPC 
    Q: np.ndarray = np.diag([5., 5., 10., 10., 10., 50., 0.01, 0.01, 0.2, 0.2, 0.2, 0.2, 0.])
    R: np.ndarray = np.diag([1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5])

    # user commands:
    cmd_xvel: float = 0.05
    cmd_yvel: float = 0.0
    cmd_yaw_turn_rate: float = 0.0
    
    # the user commands are such that the robot moves forward in x axis, but the sdf has +y as forward & +x as right so the commands need to be swapped when they are being used