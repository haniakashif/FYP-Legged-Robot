import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rlpa_ros2')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'models'), ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    world_file    = os.path.join(pkg_share, 'worlds', 'flat_cave.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    rviz_config   = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')

    floor_elev_params   = os.path.join(pkg_share, 'config', 'floor_elev_map_20cm.yaml')
    ceiling_elev_params = os.path.join(pkg_share, 'config', 'ceiling_elev_map_20cm.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    urdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model_sim.urdf')
    with open(urdf_file_path, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace('package://rlpa_ros2', f'file://{pkg_share}')
    yaml_path  = os.path.join(pkg_share, 'config', 'controllers.yaml')
    robot_desc = robot_desc.replace('$(find_pkg_share rlpa_ros2)/config/controllers.yaml', yaml_path)

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}], output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-name', 'THex_Quadruped', '-topic', 'robot_description',
            '-x', '0.05', '-y', '0.0', '-z', '0.08',
            '-R', '0.0', '-P', '0.0', '-Y', '-1.57'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    load_joint_state_broadcaster      = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'],          output='screen')
    load_imu_sensor_broadcaster        = Node(package='controller_manager', executable='spawner', arguments=['imu_sensor_broadcaster'],            output='screen')
    load_joint_group_position_controller = Node(package='controller_manager', executable='spawner', arguments=['joint_group_position_controller'], output='screen')

    rl_obs          = Node(package='rlpa_ros2', executable='rl_obs',          parameters=[{'use_sim_time': True}], output='screen')
    rl_policy       = Node(package='rlpa_ros2', executable='rl_policy',       parameters=[{'use_sim_time': True}], output='screen')
    rl_action       = Node(package='rlpa_ros2', executable='rl_action',       parameters=[{'use_sim_time': True}], output='screen')
    flight_recorder = Node(package='rlpa_ros2', executable='flight_recorder', parameters=[{'use_sim_time': True}], output='screen')

    # ── Advanced Perception Pipeline ─────────────────────────────────────────
    # Step 1: Filter raw pointcloud into floor (/filtered_points) and
    #         ceiling (/ceiling_points) streams.
    ceiling_filter = Node(
        package='rlpa_ros2', executable='ceiling_filter_node',
        name='ceiling_filter_node',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # Step 2a: Floor elevation map — elevation_mapping on /filtered_points
    floor_elevation_map = Node(
        package='elevation_mapping', executable='elevation_mapping',
        name='floor_elevation_mapping',
        parameters=[floor_elev_params, {'use_sim_time': True}],
        output='screen'
    )

    # Step 2b: Flip ceiling pointcloud Z so it looks like a floor to elevation_mapping
    ceiling_flipper = Node(
        package='rlpa_ros2', executable='ceiling_flipper_node',
        name='ceiling_flipper_node',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # Step 2c: Ceiling elevation map — elevation_mapping on /ceiling_points_flipped,
    #          remapped to /ceiling_elevation_map
    ceiling_elevation_map = Node(
        package='elevation_mapping', executable='elevation_mapping',
        name='ceiling_elevation_mapping',
        parameters=[ceiling_elev_params, {'use_sim_time': True}],
        remappings=[('elevation_map', 'ceiling_elevation_map')],
        output='screen'
    )

    # Step 2d: Negate the ceiling GridMap elevations so they represent true ceiling heights
    map_flipper = Node(
        package='rlpa_ros2', executable='map_flipper_node',
        name='map_flipper_node',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # Step 3: APF planner — syncs floor + ceiling GridMaps, runs EDT + APF,
    #         publishes [height, sprawl] on /perception/spatial_commands
    apf_perception = Node(
        package='rlpa_ros2', executable='apf_perception_node',
        name='apf_perception_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    return LaunchDescription([
        gz_resource_path, gazebo, spawn_robot, rsp_node, bridge,
        load_joint_state_broadcaster, load_imu_sensor_broadcaster,
        load_joint_group_position_controller,
        rl_obs, rl_policy, rl_action, flight_recorder,
        # Perception pipeline (order matters for topic availability)
        ceiling_filter,
        floor_elevation_map,
        ceiling_flipper,
        ceiling_elevation_map,
        map_flipper,
        apf_perception,
        rviz2,
    ])
