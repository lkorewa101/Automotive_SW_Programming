#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 획득
    package_dir = get_package_share_directory('rviz_tutorial')
    rviz_config_path = os.path.join(package_dir, 'config', 'rviz_config.rviz')

    return LaunchDescription([
        # TF 게시 노드
        Node(package='rviz_tutorial', executable='tf_publisher', name='tf_publisher', output='screen'),

        # Point Cloud 게시 노드
        Node(package='rviz_tutorial', executable='pointcloud_publisher', name='pointcloud_publisher', output='screen'),

        # Marker 게시 노드
        Node(package='rviz_tutorial', executable='marker_publisher', name='marker_publisher', output='screen'),

        # RViz2 실행 (설정 파일 포함)
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config_path], output='screen'),
    ])