from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_launch_tutorial'

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
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@email.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_publisher = my_launch_tutorial.simple_publisher:main',
            'simple_subscriber = my_launch_tutorial.simple_subscriber:main',
            'param_publisher = my_launch_tutorial.param_publisher:main',
            'sensor_module = my_launch_tutorial.sensor_module:main',
            'control_module = my_launch_tutorial.control_module:main',
        ],
    },
)
