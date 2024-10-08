#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type     = LaunchConfiguration('channel_type',     default = 'serial')
    serial_port      = LaunchConfiguration('serial_port',      default = '/dev/rplidar')
    serial_baudrate  = LaunchConfiguration('serial_baudrate',  default = '115200')
    frame_id         = LaunchConfiguration('frame_id',         default = 'laser')
    inverted         = LaunchConfiguration('inverted',         default = 'false')
    angle_compensate = LaunchConfiguration('angle_compensate', default = 'true')
    scan_mode        = LaunchConfiguration('scan_mode',        default = 'Sensitivity')





    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_transform_publisher',
            arguments=[
                '--x', '0.000',
                '--y', '0.000',
                '--z', '0.45',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'lidar',
            ]
        ),


        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{'channel_type':     channel_type,
                         'serial_port':      serial_port,
                         'serial_baudrate':  serial_baudrate,
                         'frame_id':         frame_id,
                         'inverted':         inverted,
                         'angle_compensate': angle_compensate}]),
    ])
