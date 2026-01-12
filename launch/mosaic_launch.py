from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # mosaic launch arguments
    config_arg = DeclareLaunchArgument('config', default_value='./mosaic_config.yaml', description='Path to the YAML configuration file')
    mosaic_log_level_arg = DeclareLaunchArgument('mosaic_log_level', default_value='info', description='MOSAIC Lib log level (options: debug, warning, info, error)')
    webrtc_log_level_arg = DeclareLaunchArgument('webrtc_log_level', default_value='none', description='WebRTC log level (options: none, verbose, warning, info, error)')

    mosaic_node = Node(
        package='mosaic-ros2',
        executable='mosaic_ros2_main',
        name='mosaic_ros2_node',
        parameters=[
            LaunchConfiguration('config'),
            LaunchConfiguration('mosaic_log_level'),
            LaunchConfiguration('webrtc_log_level'),
        ]
    )

    return LaunchDescription([
        config_arg,
        mosaic_log_level_arg,
        webrtc_log_level_arg,
        mosaic_node,
    ])
