from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    sensor_module = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                'path/to/sensor_module.launch.py'
            )
        ),
        launch_arguments={
            'namespace': 'robot/sensors'
        }.items()
    )

    return LaunchDescription([
        sensor_module,
    ])