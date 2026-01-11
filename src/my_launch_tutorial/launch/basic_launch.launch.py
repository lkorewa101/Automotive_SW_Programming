from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_launch_tutorial',
            executable='simple_publisher',
            name='my_publisher',
            output='screen'
        ),
        Node(
            package='my_launch_tutorial',
            executable='simple_subscriber',
            name='my_subscriber',
            output='screen'
        )
    ])