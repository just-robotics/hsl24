import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'localization'

    # parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')]

    localization = Node(
        package=package_name,
        executable='localization',
        # parameters=parameters,
        output='screen',
    )

    return LaunchDescription([
        localization,
    ])
