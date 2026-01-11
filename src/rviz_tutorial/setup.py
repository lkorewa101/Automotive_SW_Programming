from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rviz_tutorial'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@email.com',
    description='RViz2 시각화 튜토리얼 패키지',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tf_publisher = rviz_tutorial.tf_publisher:main',
            'pointcloud_publisher = rviz_tutorial.pointcloud_publisher:main',
            'marker_publisher = rviz_tutorial.marker_publisher:main',
        ],
    },
)
