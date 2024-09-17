import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'dynamic_detector'

    parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params_realsense.yaml')]

    dynamic_detector = Node(
        package=package_name,
        executable='dynamic_detector',
        parameters=parameters,
        output='screen',
    )

    obstacles_tracker = Node(
        package=package_name,
        executable='obstacles_tracker',
        parameters=parameters,
    )

    return LaunchDescription([
        dynamic_detector,
        obstacles_tracker,
    ])
