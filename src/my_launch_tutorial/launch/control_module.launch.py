from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_launch_tutorial',
            executable='simple_subscriber',
            name='temp_controller',
            namespace='robot/control',
            output='screen'
        ),
        Node(
            package='my_launch_tutorial',
            executable='simple_subscriber',
            name='pressure_controller',
            namespace='robot/control',
            output='screen'
        ),
    ])