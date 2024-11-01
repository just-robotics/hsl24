# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = "/workspace/src/bereg_nav"
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')

    # Create our own temporary YAML files that include substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    # Nodes launching commands

    start_map_server = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params])
        ])

    # If the provided param file doesn't have slam_toolbox params, we must remove the 'params_file'
    # LaunchConfiguration, or it will be passed automatically to slam_toolbox and will not load
    # the default file
    has_slam_toolbox_params = HasNodeParams(source_file=params_file,
                                            node_name='slam_toolbox')

    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(has_slam_toolbox_params))

    start_slam_toolbox_cmd_with_params = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': params_file}.items(),
        condition=IfCondition(has_slam_toolbox_params))

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_server)

    # Running SLAM Toolbox (Only one of them will be run)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_slam_toolbox_cmd_with_params)

    return ld