import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


package_name = 'master'

slam_toolobox_config = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')
rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'slam_toolbox.rviz')

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

rviz = Node(
    package='rviz2',
    namespace='',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config]
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(lidar)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz)
    
    return ld