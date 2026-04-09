import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from scipy.signal import lfilter

# Feature Flag
APPLY_FILTER = False 

# --- CONFIGURATION ---
CSV_FILE = 'adc_calibration_log_5.csv'
YAML_FILE = 'calibration_curves_2/joint_calibration.yaml'
OUTPUT_DIR = 'state_tracking_plots_' + CSV_FILE.split('.')[0][-1]  + '_' + YAML_FILE.split('/')[0][-1]  # e.g., 'state_tracking_plots_5_2'

OUTPUT_DIR = OUTPUT_DIR + ('_filtered' if APPLY_FILTER else '_raw')


# Equiripple filter coefficients (6-tap FIR)
COEFFS = [
    0.035445281711098, -0.139943584023538, 0.609060988977394, 
    0.609060988977394, -0.139943584023538, 0.035445281711098
]

JOINTS = [
    "fr_hip", "fr_knee", "fr_foot",
    "br_hip", "br_knee", "br_foot",
    "fl_hip", "fl_knee", "fl_foot",
    "bl_hip", "bl_knee", "bl_foot"
]

def load_calibration_data(filepath):
    """Safely loads the YAML configuration file."""
    try:
        with open(filepath, 'r') as file:
            # safe_load mitigates arbitrary code execution vulnerabilities in YAML parsing
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: Calibration file '{filepath}' not found.")
        return None
    except yaml.YAMLError as exc:
        print(f"Error parsing YAML: {exc}")
        return None

def main():
    # 1. Load Data
    try:
        df = pd.read_csv(CSV_FILE)
    except FileNotFoundError:
        print(f"Error: Could not find '{CSV_FILE}'.")
        return

    calib_data = load_calibration_data(YAML_FILE)
    if not calib_data:
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    time = df['timestamp'].values
    
    print(f"Generating tracking plots. FIR Filter applied: {APPLY_FILTER}")

    # 2. Process and Plot Each Joint
    for joint in JOINTS:
        rad_col = f"RAD_{joint}"
        volt_col = f"VOLT_{joint}"

        if rad_col not in df.columns or volt_col not in df.columns:
            continue

        if joint not in calib_data:
            print(f"Warning: No calibration parameters found for {joint} in YAML. Skipping.")
            continue

        # Extract mapping parameters
        slope = calib_data[joint]['slope']
        intercept = calib_data[joint]['intercept']

        # Extract raw data
        commanded_theta = df[rad_col].values
        raw_voltage = df[volt_col].values

        # Calculate actual (estimated) state from raw voltage
        actual_theta_raw = (raw_voltage * slope) + intercept

        # Setup plot
        fig, ax = plt.subplots(figsize=(10, 5))
        
        # Plot Commanded Angle
        ax.plot(time, commanded_theta, color='blue', linewidth=2, linestyle='--', label='Commanded Trajectory')
        
        if APPLY_FILTER:
            # Filter the raw voltage FIRST, then map to angle
            filtered_voltage = lfilter(COEFFS, 1.0, raw_voltage)
            actual_theta_filtered = (filtered_voltage * slope) + intercept
            
            # Plot raw state as background, filtered state as foreground
            ax.plot(time, actual_theta_raw, color='red', alpha=0.3, linewidth=1, label='Raw Estimated State')
            ax.plot(time, actual_theta_filtered, color='darkred', linewidth=2, label='Filtered Estimated State')
        else:
            # Plot only raw state
            ax.plot(time, actual_theta_raw, color='red', linewidth=1.5, label='Raw Estimated State')

        # Formatting
        ax.set_title(f"{joint.upper()} - Position Tracking Performance")
        ax.set_xlabel("Time (Seconds)")
        ax.set_ylabel("Angle (Radians)")
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend(loc='upper right')
        
        plt.tight_layout()
        
        # Save
        filename = f"{joint}_tracking.png"
        filepath = os.path.join(OUTPUT_DIR, filename)
        plt.savefig(filepath, dpi=150)
        plt.close(fig)

    print(f"\nAll plots saved to '{OUTPUT_DIR}'.")

if __name__ == '__main__':
    main()