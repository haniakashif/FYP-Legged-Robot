import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. DYNAMIC PATH FINDING
    pkg_share = get_package_share_directory('rlpa_ros2')

    # 2. CONFIGURE GAZEBO PATHS
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'models'),
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    # 3. SETUP WORLD FILE 
    world_file = os.path.join(pkg_share, 'worlds', 'my_cave_model', 'model.sdf')

    # 4. LAUNCH GAZEBO ITSELF
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(), 
    )

    # --- 5. SETUP ROBOT DESCRIPTION & INJECT PATH ---
    urdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model_sim.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Dynamically inject the absolute path to the YAML file into the URDF string
    yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    robot_desc = robot_desc.replace('$(find_pkg_share rlpa_ros2)/config/controllers.yaml', yaml_path)

    # Launch the node that holds the robot_description string
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        output='screen'
    )

    # --- 6. SPAWN THE ROBOT ---
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'THex_Quadruped',
            '-topic', 'robot_description',
            '-x', '12.76', '-y', '3.53', '-z', '-2.47',
            '-R', '0.12', '-P', '0.04', '-Y', '0.09'  
        ],
        output='screen'
    )

    # 7. BRIDGE
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'expand_gz_topic_names': True
        }],
        output='screen'
    )

    # --- 8. ROS2_CONTROL SPAWNERS ---
    
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"],
        output="screen",
    )

    load_joint_group_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller"],
        output="screen",
    )

    # --- 9. RL STACK NODES ---

    rl_obs = Node(
        package='rlpa_ros2',
        executable='rl_obs',
        name='rl_obs',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rl_policy = Node(
        package='rlpa_ros2',
        executable='rl_policy',
        name='rl_policy',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rl_action = Node(
        package='rlpa_ros2',
        executable='rl_action',
        name='rl_action',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    flight_recorder = Node(
        package='rlpa_ros2',
        executable='flight_recorder',
        name='flight_recorder',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        spawn_robot,
        rsp_node,                             
        bridge,
        load_joint_state_broadcaster,     
        load_imu_sensor_broadcaster,     
        load_joint_group_position_controller,  
        rl_obs,
        rl_policy,
        rl_action,
        # flight_recorder
    ])