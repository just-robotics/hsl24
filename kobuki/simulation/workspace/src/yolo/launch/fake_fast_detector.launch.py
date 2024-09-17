import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'yolo'

    parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')]

    fake_fast_detector = Node(
        package=package_name,
        executable='fake_fast_detector',
        parameters=parameters,
        output='screen',
    )

    return LaunchDescription([
        fake_fast_detector,
    ])
