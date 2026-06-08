import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cpg_ros2')
    quad_interface_share = get_package_share_directory('quad_interface')

    urdf_file_path = os.path.join(pkg_share, 'models', 'THex_Quadruped', 'model_hw.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    calib_yaml_path = os.path.join(quad_interface_share, 'config', 'calibration.yaml')
    robot_desc = robot_desc.replace('$(find_pkg_share quad_interface)/config/calibration.yaml', calib_yaml_path)

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen'
    )

    controllers_yaml_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    controller_manager_node = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controllers_yaml_path],
        output='screen'
    )

    load_jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], output="screen")
    load_jgpc = Node(package="controller_manager", executable="spawner", arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager"], output="screen")

    cpg_node = Node(
        package='cpg_ros2', executable='cpg_node',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        rsp_node, controller_manager_node, load_jsb, load_jgpc, cpg_node
    ])