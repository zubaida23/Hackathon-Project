from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ros2_module')

    return LaunchDescription([
        # Sensor publisher node
        Node(
            package='ros2_module',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen',
            parameters=[]
        ),

        # Robot controller node
        Node(
            package='ros2_module',
            executable='robot_controller',
            name='robot_controller',
            output='screen',
            parameters=[]
        ),

        # Command interpreter node
        Node(
            package='ros2_module',
            executable='command_interpreter',
            name='command_interpreter',
            output='screen',
            parameters=[]
        )
    ])