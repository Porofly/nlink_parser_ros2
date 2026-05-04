import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PARAM_CASTS = {
    'port_name': str,
    'baudrate': int,
    'frame_id': str,
    'serial_read_rate_hz': float,
}


def _build_node(context, *args, **kwargs):
    pkg_share = get_package_share_directory('nlink_parser_ros2')
    default_param_file = os.path.join(pkg_share, 'params', 'linktrack_aoa_init_params.yaml')
    params_file = LaunchConfiguration('params_file').perform(context) or default_param_file

    overrides = {}
    for name, cast in _PARAM_CASTS.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw:
            overrides[name] = cast(raw)

    return [
        Node(
            package='nlink_parser_ros2',
            executable='linktrack_aoa',
            name='linktrack_aoa_ros2',
            output='screen',
            parameters=[params_file, overrides],
        )
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('port_name', default_value=''),
        DeclareLaunchArgument('baudrate', default_value=''),
        DeclareLaunchArgument('frame_id', default_value=''),
        DeclareLaunchArgument('serial_read_rate_hz', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_build_node)])
