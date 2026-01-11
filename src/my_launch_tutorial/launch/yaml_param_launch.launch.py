import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지의 config 디렉토리 경로 가져오기
    config_dir = get_package_share_directory('my_launch_tutorial')
    config_file = os.path.join(config_dir, 'config', 'node_config.yaml')
    
    return LaunchDescription([
        Node(
            package='my_launch_tutorial',
            executable='param_publisher',
            name='param_publisher',
            parameters=[config_file],
            output='screen'
        )
    ])