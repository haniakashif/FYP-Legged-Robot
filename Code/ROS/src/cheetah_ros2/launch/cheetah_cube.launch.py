import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    pkg_name = 'cheetah_ros2'
    pkg_share = FindPackageShare(pkg_name)
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # CONFIGURE GAZEBO PATHS
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([pkg_share, 'models']),
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )
    
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            '/opt/gz_control_ws/install/gz_ros2_control/lib',
            ':',
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    sdf_file_path = PathJoinSubstitution([pkg_share, 'models', 'THex_Quadruped', 'model.sdf'])
    cube_sdf_file_path = PathJoinSubstitution([pkg_share, 'models', 'Cube', 'model.sdf'])
    world_file_path = PathJoinSubstitution([pkg_share, 'worlds', 'friction_world.sdf']) 
    urdf_file_path = PathJoinSubstitution([pkg_share, 'models', 'THex_Quadruped', 'model.urdf'])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_file_path]}.items(),
    )

    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'Cube',
            '-file', cube_sdf_file_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # <joint name="base_lock_joint" type="fixed">
    # <parent>world</parent>
    # <child>base_link</child>
    # </joint>

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'THex_Quadruped',
            '-file', sdf_file_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    delay_robot_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_cube,
            on_exit=[spawn_robot],
        )
    )

    robot_desc = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }],
        output='both'
    )
    
    # --- ROS-GZ Bridge ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # --- Contact Sensors Bridge (Gazebo -> ROS 2) ---
    contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/friction_world/model/THex_Quadruped/link/fl_foot/sensor/fl_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/friction_world/model/THex_Quadruped/link/fr_foot/sensor/fr_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/friction_world/model/THex_Quadruped/link/bl_foot/sensor/bl_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/friction_world/model/THex_Quadruped/link/br_foot/sensor/br_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        ],
        remappings=[
            ('/world/friction_world/model/THex_Quadruped/link/fl_foot/sensor/fl_contact/contact', '/contact/fl'),
            ('/world/friction_world/model/THex_Quadruped/link/fr_foot/sensor/fr_contact/contact', '/contact/fr'),
            ('/world/friction_world/model/THex_Quadruped/link/bl_foot/sensor/bl_contact/contact', '/contact/bl'),
            ('/world/friction_world/model/THex_Quadruped/link/br_foot/sensor/br_contact/contact', '/contact/br')
        ],
        output='screen'
    )
    
    # --- Odometry Bridge (Gazebo Ground Truth -> ROS 2) ---
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/THex_Quadruped/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    # --- Controller Spawners ---
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    load_imu_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster'],
        output='screen'
    )

    load_leg_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_effort_controller'], 
        output='screen'
    )

    delay_leg_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_leg_controller, load_imu_broadcaster],
        )
    )

    # =========================================================
    # --- CUSTOM CHEETAH ARCHITECTURE NODES ---
    # =========================================================
    
    estimator_node = Node(
        package=pkg_name, executable='estimator_node',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    
    gait_node = Node(
        package=pkg_name, executable='gait_node',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    
    fsm_node = Node(
        package=pkg_name, executable='fsm_node',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    
    stance_controller = Node(
        package=pkg_name, executable='stance_controller',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    
    swing_controller = Node(
        package=pkg_name, executable='swing_controller',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )
    
    effort_controller = Node(
        package=pkg_name, executable='effort_controller',
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )

    # =========================================================
    # --- RVIZ VISUALIZATION (OPTIONAL) ---
    # =========================================================
    
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'rviz_config.rviz'])
    rviz_visualization = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        gz_resource_path,
        gz_plugin_path,
        gazebo_sim,
        spawn_cube,
        delay_robot_spawn,
        robot_state_publisher,
        clock_bridge,
        contact_bridge,
        odom_bridge,
        load_joint_state_broadcaster,
        delay_leg_controller,
        
        # Add custom nodes to launch description
        estimator_node,
        gait_node,
        fsm_node,
        stance_controller,
        swing_controller,
        effort_controller,

        rviz_visualization
    ])