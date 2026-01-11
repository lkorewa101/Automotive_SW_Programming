from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_launch_tutorial',
            executable='simple_publisher',
            name='temp_sensor',
            namespace='robot/sensors',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
        Node(
            package='my_launch_tutorial',
            executable='simple_publisher',
            name='pressure_sensor',
            namespace='robot/sensors',
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
    ])