import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('robot_caves')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_share, 'models'),
            ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    world_file = os.path.join(pkg_share, 'worlds', 'rough_cave.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
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

    robot_desc = robot_desc.replace('package://rlpa_ros2', f'file://{pkg_share}')

    yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    robot_desc = robot_desc.replace('$(find_pkg_share rlpa_ros2)/config/controllers.yaml', yaml_path)

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

    # NOTE: Adjust spawn pose to match a valid start location in rough_cave.sdf
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'THex_Quadruped',
            '-topic', 'robot_description',
            '-x', '-2.59', '-y', '-4.65', '-z', '0.6',
            '-R', '0.0', '-P', '0.0', '-Y', '1.5'
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

    rl_obs = Node(
        package='rlpa_ros2', executable='rl_obs', name='rl_obs',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    rl_policy = Node(
        package='rlpa_ros2', executable='rl_policy', name='rl_policy',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    rl_action = Node(
        package='rlpa_ros2', executable='rl_action', name='rl_action',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    perception_preprocessor = Node(
        package='rlpa_ros2', executable='perception_preprocessor', name='perception_preprocessor',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    shared_map_params = {
        'use_sim_time': True,
        'map_frame_id': 'base_link',
        'robot_base_frame_id': 'base_link',
        'track_point_frame_id': 'base_link',
        'robot_odom_topic': '',
        'length_in_x': 4.0,
        'length_in_y': 4.0,
        'resolution': 0.05,
        'min_variance': 0.001,
        'max_variance': 0.05,
        'time_tolerance': 1.0,
        'enable_visibility_cleanup': True,
        'visibility_cleanup_rate': 1.0,
        'scanning_duration': 0.5,
        'enable_continuous_cleanup': True,
    }

    floor_mapping = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping_floor',
        parameters=[
            shared_map_params,
            {
                'input_sources': ['front_lidar'],
                'front_lidar.type': 'pointcloud',
                'front_lidar.topic': '/filtered_points',
                'front_lidar.queue_size': 10,
                'front_lidar.publish_on_update': True,
                'front_lidar.sensor_processor.type': 'perfect'
            }
        ],
        output='screen'
    )

    ceiling_mapping = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping_ceil',
        parameters=[
            shared_map_params,
            {
                'input_sources': ['ceiling_lidar'],
                'ceiling_lidar.type': 'pointcloud',
                'ceiling_lidar.topic': '/ceiling_points_flipped',
                'ceiling_lidar.queue_size': 10,
                'ceiling_lidar.publish_on_update': True,
                'ceiling_lidar.sensor_processor.type': 'perfect'
            }
        ],
        remappings=[('elevation_map', '/ceiling_elevation_map')],
        output='screen'
    )

    apf_planner = Node(
        package='rlpa_ros2', executable='apf_planner', name='apf_planner',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    camera_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_bridge',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '1.5708', '--pitch', '0', '--yaw', '-1.5708',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'THex_Quadruped/base_link/rgbd_camera'
        ],
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
        perception_preprocessor,
        floor_mapping,
        ceiling_mapping,
        apf_planner,
        camera_tf_bridge,
        rviz2
    ])
