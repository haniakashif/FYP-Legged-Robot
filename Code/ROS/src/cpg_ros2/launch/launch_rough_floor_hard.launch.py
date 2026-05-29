import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('cpg_ros2')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'models'),
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    world_file = os.path.join(pkg_share, 'worlds', 'rough_floor_hard.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    urdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model_sim.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    robot_desc = robot_desc.replace('$(find_pkg_share cpg_ros2)/config/controllers.yaml', yaml_path)

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
            '-topic', 'robot_description',
            '-x', '5.0', '-y', '-0.8', '-z', '0.6',
            '-R', '0.0', '-P', '0.0', '-Y', '1.57'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'expand_gz_topic_names': True
        }],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_joint_group_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller'],
        output='screen',
    )

    cpg_node = Node(
        package='cpg_ros2',
        executable='cpg_node',
        name='cpg_node',
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
        load_joint_group_position_controller,
        cpg_node
    ])
