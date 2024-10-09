import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("navigation"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_static_transform_publisher',
        arguments=[
            '--x', '0.000',
            '--y', '0.000',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint',
        ]
    )
    
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform_publisher',
        arguments=[
            '--x', '0.050',
            '--y', '-0.120',
            '--z', '0.300',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'camera_link',
        ]
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform_publisher',
        arguments=[
            '--x', '0.000',
            '--y', '0.000',
            '--z', '0.45',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '3.14',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'lidar',
        ]
    )
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('navigation'), 'launch', 'rviz.launch.py')]),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(base_tf)
    ld.add_action(camera_tf)
    ld.add_action(lidar_tf)
    #ld.add_action(rviz)

    return ld
