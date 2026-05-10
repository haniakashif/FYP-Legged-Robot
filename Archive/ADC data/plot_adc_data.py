import pandas as pd
import matplotlib.pyplot as plt
import os

# --- CONFIGURATION ---
CSV_FILE = 'adc_calibration_log_pcb_2.csv'
SUFFIX = "pcb_" + CSV_FILE.split('.')[0][-1] if CSV_FILE.startswith('adc_calibration_log_pcb_') else CSV_FILE.split('.')[0][-1]
OUTPUT_DIR = 'joint_plots_' + SUFFIX

# The exact names of your 12 joints
JOINTS = [
    "fr_hip", "fr_knee", "fr_foot",
    "br_hip", "br_knee", "br_foot",
    "fl_hip", "fl_knee", "fl_foot",
    "bl_hip", "bl_knee", "bl_foot"
]

def main():
    # 1. Load the data
    try:
        df = pd.read_csv(CSV_FILE)
    except FileNotFoundError:
        print(f"Error: Could not find '{CSV_FILE}'.")
        return

    # 2. Create the output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # Extract the common time axis
    time = df['timestamp']

    print(f"Loaded {len(df)} rows of data. Generating plots...")

    # 3. Generate a plot for each joint
    for joint in JOINTS:
        rad_col = f"RAD_{joint}"
        volt_col = f"VOLT_{joint}"

        # Safety check to ensure columns exist in the CSV
        if rad_col not in df.columns or volt_col not in df.columns:
            print(f"Warning: Missing data columns for {joint}. Skipping.")
            continue

        # Create a figure with 1 row and 2 columns
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # --- Left Subplot: Commanded Angle ---
        ax1.plot(time, df[rad_col], color='blue', linewidth=2, label='Target Radian')
        ax1.set_title(f"{joint.upper()} - Commanded Angle")
        ax1.set_xlabel("Time (Seconds)")
        ax1.set_ylabel("Angle (Radians)")
        ax1.grid(True, linestyle='--', alpha=0.7)
        ax1.legend()

        # --- Right Subplot: Raw Voltage Reading ---
        ax2.plot(time, df[volt_col], color='red', linewidth=1.5, label='ADC Readout')
        ax2.set_title(f"{joint.upper()} - Raw Voltage Feedback")
        ax2.set_xlabel("Time (Seconds)")
        ax2.set_ylabel("Voltage (V)")
        ax2.grid(True, linestyle='--', alpha=0.7)
        ax2.legend()

        # Overall figure formatting
        plt.suptitle(f"Step Response & Feedback Analysis: {joint}", fontsize=14, fontweight='bold')
        plt.tight_layout()

        # Save the figure to the folder and close it to free up memory
        filename = f"{joint}_response.png"
        filepath = os.path.join(OUTPUT_DIR, filename)
        plt.savefig(filepath, dpi=150)
        plt.close(fig)
        
        print(f"Saved: {filename}")

    print(f"\nSuccess! All 12 plots have been saved to the '{OUTPUT_DIR}' directory.")

if __name__ == '__main__':
    main()
