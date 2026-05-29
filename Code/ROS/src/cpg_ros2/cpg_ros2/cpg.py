import numpy as np
from typing import Tuple

class CPGController:
    def __init__(self):
        # evolved params
        self.gamma = 0.33242948003377987
        self.duty_cycle = 0.4172964738872599
        self.coupling_w = 1.814359554786942
        self.mu_r0 = 0.5977562499535333
        self.mu_o0 = -0.06821011367710861
        self.psi_1 = 0.6283185307179586
        self.mu_r1 = 0.2878435438298354
        self.mu_o1 = 0.4279269256411492
        self.psi_2 = -0.47850751097630134
        self.mu_r2_1 = 0.697308337055854
        self.mu_r2_2 = 0.007488975240002193
        self.mu_o2 = 0.85
        
        self.omega = 0.25  # Hz
        self.target_offsets = np.array([0.0, 0.5, 0.25, 0.75]) * 2 * np.pi
        self.n_legs = 4
        
        self.lower_limits = np.array([
            -0.7853, -1.5707, -1.5707,  # BL
            -0.7853, -1.5707, -1.5707,  # BR
            -0.7853, -1.5707, -1.5707,  # FL
            -0.7853, -1.5707, -1.5707   # FR
        ])
        
        self.upper_limits = np.array([
            0.7853, 1.5707, 1.5707,     # BL
            0.7853, 1.5707, 1.5707,     # BR
            0.7853, 1.5707, 1.5707,     # FL
            0.7853, 1.5707, 1.5707      # FR
        ])
        
    
    def update_state_variables(self, current_val: np.ndarray, target_val: np.ndarray, dt: float) -> np.ndarray:
        derivative = self.gamma * (target_val - current_val)
        return current_val + (derivative * dt)

    def update_global_phases(self, current_phi_0: np.ndarray, dt: float) -> np.ndarray:
        n_legs = len(current_phi_0)
        d_phi = np.zeros(n_legs)
        
        for i in range(n_legs):
            coupling_sum = 0
            for j in range(n_legs):
                if i == j: continue
                # eq 12
                phase_diff = self.target_offsets[j] - self.target_offsets[i]
                coupling_sum += self.coupling_w * np.sin(current_phi_0[j] - current_phi_0[i] - phase_diff)
            
            d_phi[i] = 2 * np.pi * self.omega + coupling_sum
            
        return current_phi_0 + (d_phi * dt)

    def compute_intra_leg_phases(self, phi_0: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        phi_1 = phi_0 + self.psi_1
        phi_2 = phi_1 + self.psi_2
        return phi_1, phi_2

    def apply_duty_cycle_filter(self, phi: np.ndarray) -> np.ndarray:
        phi_2pi = np.mod(phi, 2 * np.pi)
        res = np.zeros_like(phi)
        
        # Stance phase
        mask_stance = phi_2pi < (2 * np.pi * self.duty_cycle)
        res[mask_stance] = phi_2pi[mask_stance] / (2 * self.duty_cycle)
        
        # Swing phase
        mask_swing = ~mask_stance
        res[mask_swing] = (phi_2pi[mask_swing] + 2 * np.pi * (1 - 2 * self.duty_cycle)) / (2 * (1 - self.duty_cycle))
        
        return res

    def apply_spline_filter(self, phi_warped: np.ndarray) -> np.ndarray:
        # normalize phase eq 11
        phi_N = 2 * ((phi_warped / (2 * np.pi)) % 0.5)
        res = np.zeros_like(phi_N)
        
        mask = phi_N < 0.5
        
        res[mask] = -16 * (phi_N[mask]**3) + 12 * (phi_N[mask]**2)
        res[~mask] = 16 * ((phi_N[~mask] - 0.5)**3) - 12 * ((phi_N[~mask] - 0.5)**2) + 1
        
        return res

    def compute_target_angles(self, a: np.ndarray, o: np.ndarray, phi_warped: np.ndarray, is_joint_2: bool = False) -> np.ndarray:
        if not is_joint_2:
            # eq 4 for joint 0, 1
            return a * np.cos(phi_warped) + o
        else:
            # eq 8 for joint 2
            return a * phi_warped + o


    def clamp_to_joint_limits(self, angles: np.ndarray) -> np.ndarray:
        # Define limits based on the XML structure
        # Hip (Joint 0) range: +/- 0.7853
        # Knee (Joint 1) range: +/- 1.5707
        # Foot (Joint 2) range: +/- 1.5707
        
        lower_limits = np.array([
            -0.7853, -1.5707, -1.5707,  # BL Leg
            -0.7853, -1.5707, -1.5707,  # BR Leg
            -0.7853, -1.5707, -1.5707,  # FL Leg
            -0.7853, -1.5707, -1.5707   # FR Leg
        ])
        
        upper_limits = np.array([
            0.7853, 1.5707, 1.5707,     # BL Leg
            0.7853, 1.5707, 1.5707,     # BR Leg
            0.7853, 1.5707, 1.5707,     # FL Leg
            0.7853, 1.5707, 1.5707      # FR Leg
        ])

        return np.clip(angles, lower_limits, upper_limits)