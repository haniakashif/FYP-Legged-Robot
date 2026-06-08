import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from scipy.spatial.distance import cdist
from scipy.interpolate import griddata

# Initial position and orientation 
x = -0.5
y = 0.5
theta = 0.0

# Goal position
x_goal = 3.5
y_goal = 2.75
position_accuracy = 0.05

# APF parameters
zeta = 1.1547
eta = 0.0732
dstar = 0.3
Qstar = 0.75

# Parameters related to kinematic model
error_theta_max = np.deg2rad(45)
v_max = 0.2
Kp_omega = 1.5
omega_max = 0.5 * np.pi 

# Generate obstacles
o1_x = np.concatenate([np.linspace(1.5, 1.5, 100), np.linspace(1.5, 2, 100), 
                       np.linspace(2, 2, 100), np.linspace(2, 1.5, 100)]) - 1.0
o1_y = np.concatenate([np.linspace(1.5, 2, 100), np.linspace(2, 2, 100), 
                       np.linspace(2, 1.5, 100), np.linspace(1.5, 1.5, 100)]) - 1.0
obst1_points = np.vstack((o1_x, o1_y))

t_ang = np.linspace(0, np.pi/2, 100)
o2_x = np.concatenate([2 + np.sin(t_ang), np.linspace(3, 3, 100), np.linspace(3, 2, 100)])
o2_y = np.concatenate([2.5 + np.cos(t_ang), np.linspace(2.5, 3.5, 100), np.linspace(3.5, 3.5, 100)]) - 1.5
obst2_points = np.vstack((o2_x, o2_y))

obst2_path = Path(obst2_points.T)

# Compute potential field (Vectorized for Python performance)
x_vals = np.linspace(-1, 4, 100)
y_vals = np.linspace(-1, 3, 100)
x_grid, y_grid = np.meshgrid(x_vals, y_vals)
grid_points = np.c_[x_grid.ravel(), y_grid.ravel()]

# Attractive Potential over grid
dist_to_goal = np.linalg.norm(grid_points - np.array([x_goal, y_goal]), axis=1)
U_att = np.zeros_like(dist_to_goal)
mask_in = dist_to_goal <= dstar
mask_out = ~mask_in
U_att[mask_in] = 0.5 * zeta * dist_to_goal[mask_in]**2
U_att[mask_out] = dstar * zeta * dist_to_goal[mask_out] - 0.5 * zeta * dstar**2

# Repulsive Potential over grid
dist1 = np.min(cdist(grid_points, obst1_points.T), axis=1)
dist2 = np.min(cdist(grid_points, obst2_points.T), axis=1)
in_obst2 = obst2_path.contains_points(grid_points)

U_rep = np.zeros_like(dist_to_goal)

mask_obst1 = dist1 <= Qstar
# Using np.maximum to prevent division by zero exactly on the obstacle boundary
U_rep[mask_obst1] += 0.5 * eta * (1.0/np.maximum(dist1[mask_obst1], 1e-9) - 1.0/Qstar)**2

mask_obst2 = (dist2 <= Qstar) & (~in_obst2)
U_rep[mask_obst2] += 0.5 * eta * (1.0/np.maximum(dist2[mask_obst2], 1e-9) - 1.0/Qstar)**2

U_total = U_att + U_rep
U_total = U_total.reshape(x_grid.shape)

# Main simulation setup
plt.figure(1)
t_count = 0
dT = 0.1
t_max = 1000

X_path = [x]
Y_path = [y]

# Adjust threshold and normalize
U_total = np.clip(U_total, None, 5) # Equivalent to U_total(U_total > 5) = 5
U_min, U_max = U_total.min(), U_total.max()
U_total_norm = (U_total - U_min) / (U_max - U_min)

while np.linalg.norm([x_goal - x, y_goal - y]) > position_accuracy and t_count < t_max:
    current_pos = np.array([x, y])
    goal_pos = np.array([x_goal, y_goal])
    
    dist_to_goal = np.linalg.norm(current_pos - goal_pos)
    if dist_to_goal <= dstar:
        nablaU_att = zeta * (current_pos - goal_pos)
    else:
        nablaU_att = (dstar / dist_to_goal) * zeta * (current_pos - goal_pos)

    # Distances for current position
    dists1 = np.linalg.norm(obst1_points.T - current_pos, axis=1)
    obst1_idx = np.argmin(dists1)
    obst1_dist = dists1[obst1_idx]

    dists2 = np.linalg.norm(obst2_points.T - current_pos, axis=1)
    obst2_idx = np.argmin(dists2)
    obst2_dist = dists2[obst2_idx]

    nablaU_rep = np.array([0.0, 0.0])
    
    if obst1_dist <= Qstar:     
        p_star = obst1_points[:, obst1_idx]
        nablaU_rep += (eta * (1/Qstar - 1/obst1_dist) * (1 / obst1_dist**2)) * (current_pos - p_star)
        
    if obst2_dist <= Qstar and not obst2_path.contains_point((x, y)):          
        p_star = obst2_points[:, obst2_idx]
        nablaU_rep += (eta * (1/Qstar - 1/obst2_dist) * (1 / obst2_dist**2)) * (current_pos - p_star)
    
    nablaU = nablaU_att + nablaU_rep

    theta_ref = np.arctan2(-nablaU[1], -nablaU[0])
    
    # Wrap angle logic
    error_theta = theta_ref - theta
    error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi

    if abs(error_theta) <= error_theta_max:
        alpha = (error_theta_max - abs(error_theta)) / error_theta_max
        v_ref = min(alpha * np.linalg.norm(-nablaU), v_max)
    else:
        v_ref = 0.0

    omega_ref = Kp_omega * error_theta
    omega_ref = min(max(omega_ref, -omega_max), omega_max)
    
    theta = theta + omega_ref * dT
    x = x + v_ref * np.cos(theta) * dT
    y = y + v_ref * np.sin(theta) * dT

    t_count += 1
    X_path.append(x)
    Y_path.append(y)
    
    plt.cla()
    plt.gca().set_aspect('equal')
    plt.xlim([-1, 4])
    plt.ylim([-1, 3])
    plt.plot(obst1_points[0, :], obst1_points[1, :], '-r')
    plt.plot(obst2_points[0, :], obst2_points[1, :], '-r')
    plt.plot(x_goal, y_goal, 'ob')
    plt.plot(X_path, Y_path, '-b')
    plt.plot([x, x + 0.2 * np.cos(theta_ref)], [y, y + 0.2 * np.sin(theta_ref)], '-g') 
    plt.plot([x, x + 0.2 * np.cos(theta)], [y, y + 0.2 * np.sin(theta)], '-r') 
    
    plt.draw()
    plt.pause(0.001)

# Close 2D figure
plt.close(1)

t_final = t_count * dT
print(f"Travel time: {t_final:.2f} s")

# --- 3D Surface Plot ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot normalized potential field (cmap='viridis' is Python's standard alternative to MATLAB's 'parula')
surf = ax.plot_surface(x_grid, y_grid, U_total_norm, cmap='viridis', edgecolor='none', alpha=0.95)

# Interpolate Z values for the 3D path line
Z_path = griddata((x_grid.ravel(), y_grid.ravel()), U_total_norm.ravel(), (X_path, Y_path), method='linear')

# Plot 3D trajectory
ax.plot(X_path, Y_path, Z_path, '-k', linewidth=2)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('Normalized U(q)')
ax.set_title('Smoothed Potential Field with Robot Path')

# view_init maps directly to MATLAB's view(azimuth, elevation)
ax.view_init(elev=35, azim=135) 

plt.show()