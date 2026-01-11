from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('my_launch_tutorial')
    sensor_module = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sensor_module.launch.py')))
    control_module = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'control_module.launch.py')))
    return LaunchDescription([sensor_module, control_module])
