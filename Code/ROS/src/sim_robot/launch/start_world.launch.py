import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. DYNAMIC PATH FINDING
    pkg_share = get_package_share_directory('sim_robot')

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
    world_file = os.path.join(pkg_share, 'worlds', 'friction_world.sdf')

    # 4. LAUNCH GAZEBO ITSELF
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-v 4 {world_file}'}.items(),
    )

    # 5. SPAWN THE ROBOT
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'THex_Quadruped',
            '-file', os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model.sdf'),
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # 6. SPAWN THE CUBE
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'Cube',
            '-file', os.path.join(pkg_share, 'models', 'Cube', 'model.sdf'),
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 7. BRIDGE (YAML Configuration)
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

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        spawn_robot,
        spawn_cube,
        bridge
    ])