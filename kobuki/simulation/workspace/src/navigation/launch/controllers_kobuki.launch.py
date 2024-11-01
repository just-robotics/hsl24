import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'navigation'

    parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'controllers_params_kobuki.yaml')]

    controller = Node(
        package=package_name,
        executable='master_controller',
        parameters=parameters,
        output='screen',
    )

    return LaunchDescription([
        controller,
    ])
