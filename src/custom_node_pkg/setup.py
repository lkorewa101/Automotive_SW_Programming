# setup.py 파일 수정
from setuptools import setup

package_name = 'custom_node_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='ROS2 커스텀 메시지 실습 패키지',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = custom_node_pkg.SensorPublisher:main',
            'sensor_subscriber = custom_node_pkg.SensorSubscriber:main',
            'robot_status_publisher = custom_node_pkg.RobotStatusPublisher:main',
            'robot_status_subscriber = custom_node_pkg.RobotStatusSubscriber:main',
        ],
    },
)