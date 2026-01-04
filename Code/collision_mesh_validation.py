import csv
import matplotlib.pyplot as plt
import sys

def read_csv(filename):
    data = {}
    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            for key, value in row.items():
                if key not in data:
                    data[key] = []
                try:
                    data[key].append(float(value) if value != '' else None)
                except ValueError:
                    data[key].append(value if value != '' else None)
    return data

def calculate_errors(actual_data, simplified_data):
    """Calculate absolute errors between actual and simplified data."""
    errors = {}
    
    for key in actual_data.keys():
        if key == 'Time_Step':
            continue
        
        if key in simplified_data:
            actual_vals = actual_data[key]
            simplified_vals = simplified_data[key]
            
            # Calculate errors for each time step
            error_vals = []
            min_len = min(len(actual_vals), len(simplified_vals))
            
            for i in range(min_len):
                if actual_vals[i] is not None and simplified_vals[i] is not None:
                    error_vals.append(abs(actual_vals[i] - simplified_vals[i]))
                else:
                    error_vals.append(None)
            
            errors[key] = error_vals
    
    return errors

def plot_errors(joint_errors, torque_errors):
    """Plot errors in 4x3 subplot format."""
    legs = ["FR", "BR", "BL", "FL"]
    joint_types = ["hip", "knee", "foot"]
    
    # Create figure for joint state errors
    fig1, axes1 = plt.subplots(4, 3, figsize=(15, 12))
    fig1.suptitle("Joint State Errors: |Actual - Simplified|", fontsize=16)
    
    for leg_ind, leg in enumerate(legs):
        for joint_ind, joint_type in enumerate(joint_types):
            ax = axes1[leg_ind, joint_ind]
            
            # Look for matching column in errors
            key_state = f'{leg}_{joint_type}_state'
            
            if key_state in joint_errors:
                error_data = [e for e in joint_errors[key_state] if e is not None]
                if error_data:
                    ax.plot(error_data, linestyle='-', linewidth=1.5, color='blue')
                    ax.set_title(f"{leg} {joint_type}")
                    ax.set_xlabel("Time Step")
                    ax.set_ylabel("Angle Error (deg)")
                    ax.grid(True, alpha=0.3)
            else:
                ax.set_title(f"{leg} {joint_type} - No Data")
                ax.set_xlabel("Time Step")
                ax.set_ylabel("Angle Error (deg)")
    
    plt.tight_layout()
    plt.savefig("joint_state_errors.png")
    
    # Create figure for torque errors
    fig2, axes2 = plt.subplots(4, 3, figsize=(15, 12))
    fig2.suptitle("Torque Errors: |Actual - Simplified|", fontsize=16)
    
    for leg_ind, leg in enumerate(legs):
        for joint_ind, joint_type in enumerate(joint_types):
            ax = axes2[leg_ind, joint_ind]
            
            # Look for matching column in errors
            key_torque = f'{leg}_{joint_type}_torque'
            
            if key_torque in torque_errors:
                error_data = [e for e in torque_errors[key_torque] if e is not None]
                if error_data:
                    ax.plot(error_data, linestyle='-', linewidth=1.5, color='red')
                    ax.set_title(f"{leg} {joint_type}")
                    ax.set_xlabel("Time Step")
                    ax.set_ylabel("Torque Error (N⋅m)")
                    ax.grid(True, alpha=0.3)
            else:
                ax.set_title(f"{leg} {joint_type} - No Data")
                ax.set_xlabel("Time Step")
                ax.set_ylabel("Torque Error (N⋅m)")
    
    plt.tight_layout()
    plt.savefig("joint_torque_errors.png")
    plt.show()

def main():
    # if len(sys.argv) != 5:
    #     print("Usage: python3 collisionMeshValidation.py <actual_torque.csv> <simplified_torque.csv> <actual_joint_state.csv> <simplified_joint_state.csv>")
    #     sys.exit(1)
    
    # actual_torque_file = sys.argv[1]
    # simplified_torque_file = sys.argv[2]
    # actual_joint_state_file = sys.argv[3]
    # simplified_joint_state_file = sys.argv[4]
    
    actual_torque_file = "torque_data_actual_suspended.csv"
    simplified_torque_file = "torque_data_simplified_suspended.csv"
    actual_joint_state_file = "joint_data_actual_suspended.csv"
    simplified_joint_state_file = "joint_data_simplified_suspended.csv"

    print(f"Reading {actual_torque_file}...")
    actual_torque_data = read_csv(actual_torque_file)
    
    print(f"Reading {simplified_torque_file}...")
    simplified_torque_data = read_csv(simplified_torque_file)
    
    print(f"Reading {actual_joint_state_file}...")
    actual_joint_state_data = read_csv(actual_joint_state_file)
    
    print(f"Reading {simplified_joint_state_file}...")
    simplified_joint_state_data = read_csv(simplified_joint_state_file)
    
    print("Calculating errors...")
    torque_errors = calculate_errors(actual_torque_data, simplified_torque_data)
    joint_state_errors = calculate_errors(actual_joint_state_data, simplified_joint_state_data)
    
    print("Plotting errors...")
    plot_errors(joint_state_errors, torque_errors)
    
    # Calculate and print statistics
    print("\n=== Error Statistics ===")
    
    print("\nJoint State Errors (degrees):")
    for key, errors in joint_state_errors.items():
        valid_errors = [e for e in errors if e is not None]
        if valid_errors:
            avg_error = sum(valid_errors) / len(valid_errors)
            max_error = max(valid_errors)
            print(f"  {key}: Avg={avg_error:.4f}°, Max={max_error:.4f}°")
    
    print("\nTorque Errors (N⋅m):")
    for key, errors in torque_errors.items():
        valid_errors = [e for e in errors if e is not None]
        if valid_errors:
            avg_error = sum(valid_errors) / len(valid_errors)
            max_error = max(valid_errors)
            print(f"  {key}: Avg={avg_error:.4f} N⋅m, Max={max_error:.4f} N⋅m")
    
    print("\nPlots saved as 'joint_state_errors.png' and 'joint_torque_errors.png'")

if __name__ == "__main__":
    main()
