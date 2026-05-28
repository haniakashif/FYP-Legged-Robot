import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rlpa_ros2')

    world_file      = os.path.join(pkg_share, 'worlds', 'ControlTube.sdf')
    bridge_config   = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    floor_cfg       = os.path.join(pkg_share, 'config', 'floor_elev_map_20cm.yaml')
    ceiling_cfg     = os.path.join(pkg_share, 'config', 'ceiling_elev_map_20cm.yaml')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'worlds') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Bridge: Gazebo /rgbd_camera/points → ROS2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # 3. Static TF: camera_link → sensor frame (identity; camera is fixed in world)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_link', 'animated_rgbd_camera/link/rgbd_camera'],
        output='screen',
    )

    # 4. Split raw cloud → /filtered_points (floor) and /ceiling_points
    ceiling_filter = Node(
        package='rlpa_ros2',
        executable='ceiling_filter_node',
        name='ceiling_filter_node',
        output='screen',
    )

    # 5. Floor elevation map from /filtered_points → /elevation_map
    floor_elevation_map = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='floor_elevation_mapping',
        parameters=[floor_cfg, {'use_sim_time': True}],
        output='screen',
    )

    # 6. Flip ceiling cloud Z → /ceiling_points_flipped
    ceiling_flipper = Node(
        package='rlpa_ros2',
        executable='ceiling_flipper_node',
        name='ceiling_flipper_node',
        output='screen',
    )

    # 7. Ceiling elevation map from /ceiling_points_flipped → /ceiling_elevation_map
    ceiling_elevation_map = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='ceiling_elevation_mapping',
        parameters=[ceiling_cfg, {'use_sim_time': True}],
        remappings=[('elevation_map', '/ceiling_elevation_map')],
        output='screen',
    )

    # 8. Negate ceiling GridMap elevations → /ceiling_elevation_map_flipped
    map_flipper = Node(
        package='rlpa_ros2',
        executable='map_flipper_node',
        name='map_flipper_node',
        output='screen',
    )

    # 9. APF planner (debug) — consumes /elevation_map + /ceiling_elevation_map_flipped
    apf_planner2 = Node(
        package='rlpa_ros2',
        executable='apf_planner2',
        name='apf_planner_debug',
        parameters=[{
            'goal_x': 2.0,
            'goal_y': 0.0,
            'publish_debug_topics': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        bridge,
        static_tf,
        ceiling_filter,
        floor_elevation_map,
        ceiling_flipper,
        ceiling_elevation_map,
        map_flipper,
        apf_planner2,
    ])
