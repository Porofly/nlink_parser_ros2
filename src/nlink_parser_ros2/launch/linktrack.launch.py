import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'system_id',
            default_value='1',
            description='System ID to use for namespace'
        ),
    ])

    linktrack_tag0_param_file = os.path.join(
        get_package_share_directory('nlink_parser_ros2'),
        'params',
        'linktrack_init_tag0_params.yaml'
    )

    linktrack_tag1_param_file = os.path.join(
        get_package_share_directory('nlink_parser_ros2'),
        'params',
        'linktrack_init_tag1_params.yaml'
    )

    linktrack_tag0_node = Node(
        package="nlink_parser_ros2",
        executable="linktrack",
        output="screen",
        arguments=[linktrack_tag0_param_file],
        parameters=[{ 'system_id': LaunchConfiguration('system_id'), 'tag_id': 0 }]
    )
    
    linktrack_tag1_node = Node(
        package="nlink_parser_ros2",
        executable="linktrack",
        output="screen",
        arguments=[linktrack_tag1_param_file],
        parameters=[{ 'system_id': LaunchConfiguration('system_id'), 'tag_id': 1 }]
    )
    
    ld.add_action(linktrack_tag0_node)
    ld.add_action(linktrack_tag1_node)
    return ld