from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='2.0',
        description='메시지 발행 주기 (Hz)'
    )

    return LaunchDescription([
        frequency_arg,
        Node(
            package='my_launch_tutorial',
            executable='param_publisher',
            parameters=[{
                'publish_frequency': LaunchConfiguration('frequency')
            }]
        )
    ])