import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. PATH FINDING
    pkg_share = get_package_share_directory('rlpa_ros2')
    quad_interface_share = get_package_share_directory('quad_interface')

    # --- 2. SETUP ROBOT DESCRIPTION & INJECT PATHS ---
    urdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model_hw.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Dynamically inject the absolute path to your calibration YAML into the URDF string
    calib_yaml_path = os.path.join(quad_interface_share, 'config', 'calibration.yaml')
    robot_desc = robot_desc.replace('$(find_pkg_share quad_interface)/config/calibration.yaml', calib_yaml_path)

    # 3. ROBOT STATE PUBLISHER
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False  # STRICLY FALSE FOR HARDWARE
        }],
        output='screen'
    )

    # --- 4. THE STANDALONE CONTROLLER MANAGER (NEW FOR HARDWARE) ---
    # This node loads your C++ QuadHardwareInterface and runs the 50Hz read/write loop
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_desc},
            controllers_yaml_path
        ],
        output='screen'
    )

    # --- 5. ROS2_CONTROL SPAWNERS ---
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    load_imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    load_joint_group_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --- 6. RL STACK NODES ---
    rl_obs = Node(
        package='rlpa_ros2',
        executable='rl_obs',
        name='rl_obs',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    rl_policy = Node(
        package='rlpa_ros2',
        executable='rl_policy',
        name='rl_policy',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    rl_action = Node(
        package='rlpa_ros2',
        executable='rl_action',
        name='rl_action',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    flight_recorder = Node(
        package='rlpa_ros2',
        executable='flight_recorder',
        name='flight_recorder',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,                             
        controller_manager_node,
        load_joint_state_broadcaster,   
        load_imu_sensor_broadcaster,       
        load_joint_group_position_controller,  
        rl_obs,
        rl_policy,
        rl_action,
        # flight_recorder
    ])