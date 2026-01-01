from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # mosaic launch arguments
    mosaic_log_level_arg = DeclareLaunchArgument('mosaic_log_level', default_value='info', description='MOSAIC Lib log level (options: debug, warning, info, error)')
    webrtc_log_level_arg = DeclareLaunchArgument('webrtc_log_level', default_value='none', description='WebRTC log level (options: none, verbose, warning, info, error)')

    # Husky RTC Sender node
    mosaic_node = Node(
        package='mosaic-ros2',
        executable='mosaic_ros2_main',
        name='mosaic_ros2_node',
        parameters=[{
            'mosaic_log_level': LaunchConfiguration('mosaic_log_level'),
            'webrtc_log_level': LaunchConfiguration('webrtc_log_level'),
        }]
    )

    return LaunchDescription([
        mosaic_log_level_arg,
        webrtc_log_level_arg,
        mosaic_node,
    ])
