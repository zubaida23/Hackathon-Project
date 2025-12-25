import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Module 1: The Robotic Nervous System for Physical AI & Humanoid Robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = ros2_module.sensor_publisher:main',
            'robot_controller = ros2_module.robot_controller:main',
            'command_interpreter = ros2_module.command_interpreter:main',
        ],
    },
)