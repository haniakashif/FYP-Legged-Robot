import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from scipy import stats

# --- CONFIGURATION ---
CSV_FILE = 'adc_calibration_log_pcb_1.csv'
SUFFIX = "pcb_" + CSV_FILE.split('.')[0][-1] if CSV_FILE.startswith('adc_calibration_log_pcb_') else CSV_FILE.split('.')[0][-1]
OUTPUT_DIR = 'calibration_curves_' + SUFFIX
SETTLING_TIME = 1.0 # Seconds to ignore after a command change
YAML_FILENAME = 'joint_calibration.yaml'

JOINTS = [
    "bl_hip", "br_hip", "fl_hip", "fr_hip",
    "bl_knee", "br_knee", "fl_knee", "fr_knee",
    "bl_foot", "br_foot", "fl_foot", "fr_foot"
]

def main():
    try:
        df = pd.read_csv(CSV_FILE)
    except FileNotFoundError:
        print(f"Error: Could not find '{CSV_FILE}'.")
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    print(f"--- Sensor Calibration Equations (Ignoring {SETTLING_TIME}s Transient) ---")

    # Initialize a dictionary to store the calibration parameters
    calibration_params = {}

    for joint in JOINTS:
        rad_col = f"RAD_{joint}"
        volt_col = f"VOLT_{joint}"

        if rad_col not in df.columns or volt_col not in df.columns:
            continue

        # Extract data arrays
        time = df['timestamp'].values
        V_all = df[volt_col].values
        theta_all = df[rad_col].values

        # --- TRANSIENT MASKING LOGIC ---
        valid_mask = np.ones(len(df), dtype=bool)
        
        # Treat the very first timestamp as a command change for initial settling
        change_times = [time[0]] 
        
        # Detect subsequent changes in the commanded angle (> 0.0001 rad to ignore float noise)
        angle_diffs = np.abs(df[rad_col].diff().fillna(0))
        change_times.extend(df.loc[angle_diffs > 1e-4, 'timestamp'].values)

        # Apply mask: Invalidate rows within the settling time window
        for t_change in change_times:
            invalid_idx = (df['timestamp'] >= t_change) & (df['timestamp'] < t_change + SETTLING_TIME)
            valid_mask[invalid_idx] = False

        # Filter the arrays using the mask
        V_steady = V_all[valid_mask]
        theta_steady = theta_all[valid_mask]

        if len(V_steady) == 0:
            print(f"\nJoint: {joint.upper()} - Warning: Settling time too high, no data left!")
            continue

        # Perform Ordinary Least Squares (OLS) linear regression on STEADY STATE data
        slope, intercept, r_value, p_value, std_err = stats.linregress(V_steady, theta_steady)
        r_squared = r_value ** 2

        # Populate the dictionary for YAML serialization.
        # It is strictly necessary to cast numpy float64 to native Python float
        # to ensure compatibility with standard PyYAML dumping.
        calibration_params[joint] = {
            'slope': float(slope),
            'intercept': float(intercept)
        }

        min_angle, max_angle = np.min(theta_steady), np.max(theta_steady)
        min_volt, max_volt = np.min(V_steady), np.max(V_steady)

        print(f"\nJoint: {joint.upper()}")
        print(f"  Equation:   theta = {slope:.4f} * V + {intercept:.4f}")
        print(f"  Linearity:  R^2 = {r_squared:.4f}")
        print(f"  Data Used:  {len(V_steady)} / {len(V_all)} samples")

        # --- Generate Calibration Curve Plot ---
        fig, ax = plt.subplots(figsize=(8, 6))
        
        # Plot ignored transient data
        ax.scatter(V_all[~valid_mask], theta_all[~valid_mask], color='red', alpha=0.15, label='Ignored (Transient)')
        
        # Plot valid steady-state data
        ax.scatter(V_steady, theta_steady, color='blue', alpha=0.3, label='Used (Steady-State)')
        
        # Plot the line of best fit
        V_fit = np.linspace(np.min(V_all), np.max(V_all), 100)
        theta_fit = slope * V_fit + intercept
        ax.plot(V_fit, theta_fit, color='black', linewidth=2, 
                label=f'Fit: $\\theta = {slope:.3f}V + {intercept:.3f}$')
        
        ax.set_title(f"{joint.upper()} - Steady-State Calibration Curve\n(Settling Time: {SETTLING_TIME}s)")
        ax.set_xlabel("Feedback Voltage (V)")
        ax.set_ylabel("Commanded Angle (Radians)")
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend()

        filename = f"{joint}_calibration_curve.png"
        filepath = os.path.join(OUTPUT_DIR, filename)
        plt.savefig(filepath, dpi=150)
        plt.close(fig)

    # --- YAML Serialization ---
    yaml_filepath = os.path.join(OUTPUT_DIR, YAML_FILENAME)
    with open(yaml_filepath, 'w') as file:
        # default_flow_style=False enforces standard block YAML syntax rather than inline dictionaries
        # sort_keys=False maintains the hierarchical order based on your JOINTS array
        yaml.dump(calibration_params, file, default_flow_style=False, sort_keys=False)

    print(f"\nSteady-state calibration curves and '{YAML_FILENAME}' have been saved to '{OUTPUT_DIR}'.")

if __name__ == '__main__':
    main()