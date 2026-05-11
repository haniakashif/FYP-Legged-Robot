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

    world_file = os.path.join(pkg_share, 'worlds', 'my_cave_model', 'model.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz_config.rviz')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(), 
    )

    urdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model_sim.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    robot_desc = robot_desc.replace('package://rlpa_ros2', f'file://{pkg_share}')
    robot_desc = robot_desc.replace('$(find_pkg_share rlpa_ros2)/config/controllers.yaml', os.path.join(pkg_share, 'config', 'controllers.yaml'))

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', 
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}], output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'THex_Quadruped', '-topic', 'robot_description', '-x', '12.76', '-y', '3.53', '-z', '-2.47', '-R', '0.12', '-P', '0.04', '-Y', '0.09'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'expand_gz_topic_names': True}], output='screen'
    )

    load_joint_state_broadcaster = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen")
    load_imu_sensor_broadcaster = Node(package="controller_manager", executable="spawner", arguments=["imu_sensor_broadcaster"], output="screen")
    load_joint_group_position_controller = Node(package="controller_manager", executable="spawner", arguments=["joint_group_position_controller"], output="screen")

    rl_obs = Node(package='rlpa_ros2', executable='rl_obs', name='rl_obs', parameters=[{'use_sim_time': True}], output='screen')
    rl_policy = Node(package='rlpa_ros2', executable='rl_policy', name='rl_policy', parameters=[{'use_sim_time': True}], output='screen')
    rl_action = Node(package='rlpa_ros2', executable='rl_action', name='rl_action', parameters=[{'use_sim_time': True}], output='screen')

    perception_preprocessor = Node(
        package='rlpa_ros2', executable='perception_preprocessor', name='perception_preprocessor',
        parameters=[{'use_sim_time': True}], output='screen'
    )

    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    return LaunchDescription([
        gz_resource_path, gazebo, spawn_robot, rsp_node, bridge,
        load_joint_state_broadcaster, load_imu_sensor_broadcaster, load_joint_group_position_controller,  
        rl_obs, rl_policy, rl_action, perception_preprocessor, rviz2
    ])