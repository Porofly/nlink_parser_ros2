import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Type coercion for parameters that the yaml stores as non-string. Without
# this, an `int` parameter overridden from a launch argument would arrive
# as a string and ROS 2 would reject the type mismatch.
_PARAM_CASTS = {
    'port_name': str,
    'baudrate': int,
    'frame_id': str,
    'serial_read_rate_hz': float,
}


def _build_node(context, *args, **kwargs):
    pkg_share = get_package_share_directory('nlink_parser_ros2')
    default_param_file = os.path.join(pkg_share, 'params', 'linktrack_init_params.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_param_file

    # Collect only non-empty CLI overrides so yaml defaults stay in effect
    # for arguments the user did not pass.
    overrides = {}
    for name, cast in _PARAM_CASTS.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw:
            overrides[name] = cast(raw)

    return [
        Node(
            package='nlink_parser_ros2',
            executable='linktrack',
            name='linktrack_ros2',
            output='screen',
            parameters=[params_file, overrides],
        )
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('port_name', default_value='',
                              description='Serial device path; empty = use yaml.'),
        DeclareLaunchArgument('baudrate', default_value='',
                              description='UART baud rate; empty = use yaml.'),
        DeclareLaunchArgument('frame_id', default_value='',
                              description='header.frame_id; empty = use yaml.'),
        DeclareLaunchArgument('serial_read_rate_hz', default_value='',
                              description='Serial RX drain rate in Hz; empty = use yaml.'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Path to a ROS 2 parameter yaml; empty = bundled default.'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_build_node)])
