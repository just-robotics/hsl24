import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'master'

    rsp_master = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rsp_slave = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slave'), 'launch', 'rsp.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'cafe.world')
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

    goal_pose_pub = Node(
        package='graph_travel',
        executable='goal_pose_publisher',
        name='goal_pose_publisher',
        namespace='/master'
    )

    return LaunchDescription([
        rsp_master,
        rsp_slave,
        gazebo,
        spawn_entity_master,
        spawn_entity_slave,
        goal_pose_pub,
    ])
