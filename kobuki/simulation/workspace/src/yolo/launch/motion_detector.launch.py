import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'yolo'
    
    parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')]

    motion_detector = Node(
        package=package_name,
        executable='motion_detector',
        parameters=[parameters],
        output='screen',
    )

    return LaunchDescription([
        motion_detector,
    ])
