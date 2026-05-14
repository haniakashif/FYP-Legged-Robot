import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('cheetah_ros2')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'models'),
            ':',
            os.path.join(pkg_share, 'worlds'),
            ':',
            os.path.join(pkg_share, 'worlds', 'my_cave_model'),
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

    world_file = os.path.join(pkg_share, 'worlds', 'rough_floor_medium.sdf')
    sdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model.sdf')
    urdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model.urdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    robot_desc = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

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

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'THex_Quadruped',
            '-file', sdf_file_path,
            '-x', '5.0', '-y', '-0.8', '-z', '0.6',
            '-R', '0.0', '-P', '0.0', '-Y', '1.57'
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/robot_in_cave/model/THex_Quadruped/link/fl_foot/sensor/fl_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/robot_in_cave/model/THex_Quadruped/link/fr_foot/sensor/fr_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/robot_in_cave/model/THex_Quadruped/link/bl_foot/sensor/bl_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/world/robot_in_cave/model/THex_Quadruped/link/br_foot/sensor/br_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
        ],
        remappings=[
            ('/world/robot_in_cave/model/THex_Quadruped/link/fl_foot/sensor/fl_contact/contact', '/contact/fl'),
            ('/world/robot_in_cave/model/THex_Quadruped/link/fr_foot/sensor/fr_contact/contact', '/contact/fr'),
            ('/world/robot_in_cave/model/THex_Quadruped/link/bl_foot/sensor/bl_contact/contact', '/contact/bl'),
            ('/world/robot_in_cave/model/THex_Quadruped/link/br_foot/sensor/br_contact/contact', '/contact/br')
        ],
        output='screen'
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/THex_Quadruped/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen'
    )

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

    estimator_node = Node(
        package='cheetah_ros2',
        executable='estimator_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    gait_node = Node(
        package='cheetah_ros2',
        executable='gait_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fsm_node = Node(
        package='cheetah_ros2',
        executable='fsm_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    stance_controller = Node(
        package='cheetah_ros2',
        executable='stance_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    swing_controller = Node(
        package='cheetah_ros2',
        executable='swing_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    effort_controller = Node(
        package='cheetah_ros2',
        executable='effort_controller',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    rviz_visualization = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gz_plugin_path,
        gazebo,
        spawn_robot,
        rsp_node,
        clock_bridge,
        contact_bridge,
        odom_bridge,
        load_joint_state_broadcaster,
        delay_leg_controller,
        estimator_node,
        gait_node,
        fsm_node,
        stance_controller,
        swing_controller,
        effort_controller,
        rviz_visualization
    ])
