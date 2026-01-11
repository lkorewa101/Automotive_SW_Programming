from setuptools import find_packages, setup

package_name = 'my_action_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@email.com',
    description='커스텀 액션 예제 패키지',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'fibonacci_action_server = my_action_package.fibonacci_action_server:main',
        'fibonacci_action_client = my_action_package.fibonacci_action_client:main',
    ]   ,
    },
)
