import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share  = get_package_share_directory('perception')
    world_file = os.path.join(pkg_share, 'worlds', 'ControlTube.sdf')
    floor_cfg  = os.path.join(pkg_share, 'config', 'floor_elev_map.yaml')
    ceiling_cfg = os.path.join(pkg_share, 'config', 'ceiling_elev_map.yaml')

    # Point Gazebo's model:// resolver at the bundled worlds directory
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'worlds') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    # 1. Gazebo  (equivalent to tab_gazebo.sh)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Bridge: Gazebo → ROS2  (equivalent to tab_bridge.sh)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen',
    )

    # 3. Split cloud into floor (/filtered_points) and ceiling (/ceiling_points)
    ceiling_filter = Node(
        package='perception',
        executable='ceiling_filter',
        name='ceiling_filter',
        output='screen',
    )

    # 4. Floor elevation map from /filtered_points  (equivalent to tab_floormap.sh)
    floor_map = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='floor_elevation_mapping',
        parameters=[floor_cfg],
        output='screen',
    )

    # 5. Negate Z on ceiling cloud  (equivalent to tab_flipper.sh)
    ceiling_flipper = Node(
        package='perception',
        executable='ceiling_flipper',
        name='ceiling_flipper',
        output='screen',
    )

    # 6. Ceiling elevation map from /ceiling_points_flipped  (equivalent to tab_ceilingmap.sh)
    ceiling_map = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='ceiling_elevation_mapping',
        parameters=[ceiling_cfg],
        remappings=[('elevation_map', '/ceiling_elevation_map')],
        output='screen',
    )

    # 7. Negate elevation values to restore true ceiling heights  (equivalent to tab_mapflipper.sh)
    map_flipper = Node(
        package='perception',
        executable='map_flipper',
        name='ceiling_map_flipper',
        output='screen',
    )

    # 8. APF planner: clearance → safe posture → gradient-descent path
    map2apf = Node(
        package='perception',
        executable='map2apf',
        name='posture_optimizer_debug',
        output='screen',
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        bridge,
        ceiling_filter,
        floor_map,
        ceiling_flipper,
        ceiling_map,
        map_flipper,
        map2apf,
    ])
