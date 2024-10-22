import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


package_name = 'robot'

slam_toolobox_config = os.path.join(get_package_share_directory(package_name), 'config', 'slam_lidar_params.yaml')

lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(package_name), 'launch', 'lidar.launch.py')]
    ),
)

slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]
        ),
        launch_arguments={'slam_params_file': slam_toolobox_config}.items()
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(lidar)
    ld.add_action(slam_toolbox)
    
    return ld
