from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='2.0',
        description='메시지 발행 주기 (Hz)'
    )
    
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='기본',
        description='메시지 접두사'
    )

    return LaunchDescription([
        frequency_arg,
        prefix_arg,
        Node(
            package='my_launch_tutorial',
            executable='param_publisher',
            parameters=[{
                'publish_frequency': LaunchConfiguration('frequency'),
                'message_prefix': LaunchConfiguration('prefix')
            }],
            output='screen'
        )
    ])