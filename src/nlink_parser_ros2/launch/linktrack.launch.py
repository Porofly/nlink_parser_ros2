import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('nlink_parser_ros2'),
        'params', 'linktrack_init_params.yaml')

    linktrack_node = Node(
        package="nlink_parser_ros2",
        executable="linktrack",
        name="linktrack_ros2",
        output="screen",
        parameters=[param_file],
    )

    return LaunchDescription([linktrack_node])
