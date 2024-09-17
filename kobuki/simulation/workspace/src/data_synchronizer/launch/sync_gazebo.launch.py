import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'data_synchronizer'

    parameters = [os.path.join(get_package_share_directory(package_name), 'config', 'params_gazebo.yaml')]

    data_synchronizer = Node(
        package=package_name,
        executable='data_synchronizer',
        parameters=parameters,
        output='screen',
    )

    yolo_dispatcher = Node(
        package=package_name,
        executable='yolo_dispatcher',
        parameters=parameters,
    )

    return LaunchDescription([
        data_synchronizer,
        yolo_dispatcher,
    ])
