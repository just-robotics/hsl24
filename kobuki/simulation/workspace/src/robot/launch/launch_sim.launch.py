import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'robot'

    rsp_master = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp_master.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rsp_slave = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp_slave.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'maze.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'use_sim_time': 'true',
                              'world': world}.items()
    )

    spawn_entity_master = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'tb2_master',
                                   '-x', '-0.34',
                                   '-y', '-0.18',
                                   '-z', '0.23'],
                        output='screen',
                        namespace='master',
                        )

    spawn_entity_slave = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'tb2_slave',
                   '-x', '1.5',
                   '-y', '2.5',
                   '-z', '0.23'],
        output='screen',
        namespace='slave'
    )

    controller_master = Node(
        package = "navigation",
        executable = "controller",
        output = "screen",
        namespace = '/master'
    )

    warper_slave = Node(
        package = "cmd_vel_warper",
        executable = "warper",
        output = "screen",
        namespace = '/slave'
    )

    return LaunchDescription([
        rsp_master,
        rsp_slave,
        gazebo,
        spawn_entity_master,
        spawn_entity_slave,
        controller_master,
        warper_slave,
    ])
