from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # mosaic launch arguments
    mosaic_config_arg = DeclareLaunchArgument('mosaic_config', default_value='./mosaic_config.yaml',
                                              description='Path to the YAML configuration file')
    mosaic_log_level_arg = DeclareLaunchArgument('mosaic_log_level', default_value='info',
                                                 description='MOSAIC Lib log level (options: debug, warning, info, error)')
    webrtc_log_level_arg = DeclareLaunchArgument('webrtc_log_level', default_value='none',
                                                 description='WebRTC log level (options: none, verbose, warning, info, error)')

    mosaic_node = Node(
        package='mosaic-ros2-bringup',
        executable='mosaic-ros2-bringup',
        name='mosaic-ros2-bringup',
        parameters=[{
            'mosaic_config': LaunchConfiguration('mosaic_config'),
            'mosaic_log_level': LaunchConfiguration('mosaic_log_level'),
            'webrtc_log_level': LaunchConfiguration('webrtc_log_level'),
        }]
    )

    return LaunchDescription([
        mosaic_config_arg,
        mosaic_log_level_arg,
        webrtc_log_level_arg,
        mosaic_node,
    ])
