import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'tb2'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_entity_master = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'tb2_1',
                                   '-x', '-0.34',
                                   '-y', '-0.18',
                                   '-z', '0.23'],
                        output='screen',
                        namespace='master',
                        )

    spawn_entity_slave = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'tb2_2',
                   '-x', '-1.5',
                   '-y', '2.5',
                   '-z', '0.23'],
        output='screen',
        namespace='slave'
    )

    goal_pose_pub = Node(
        package='graph_travel_2',
        executable='goal_pose_publisher',
        name='goal_pose_publisher',
    )

    meeting_slave = Node(
        package='slave_meeting',
        executable='master_to_slave',
        name='master_to_slave',
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity_master,
        spawn_entity_slave,
        goal_pose_pub,
        meeting_slave,
    ])
