import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'my_robot_description'

    # Get package share directory
    pkg_share = get_package_share_directory(package_name)

    # URDF file path
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    # Load URDF content
    with open(urdf_path, 'r') as file:
        robot_desc = file.read()

    # RViz config path
    rviz_config_path = os.path.join(pkg_share, 'config', 'view_robot.rviz')

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Robot Controller
        Node(
            package='my_robot_description',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])