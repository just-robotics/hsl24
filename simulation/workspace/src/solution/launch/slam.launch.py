import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


package_name = 'solution'

slam_toolobox_config = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')
rviz_config = os.path.join(get_package_share_directory(package_name), 'config', 'slam_toolbox.rviz')

use_sim_time = LaunchConfiguration('use_sim_time')

declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]
        ),
        launch_arguments={'slam_params_file': slam_toolobox_config, 'use_sim_time' : use_sim_time}.items()
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

    ld.add_action(declare_use_sim_time_cmd)
    
    ld.add_action(slam_toolbox)
    ld.add_action(rviz)
    
    return ld