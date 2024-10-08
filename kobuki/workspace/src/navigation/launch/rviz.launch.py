#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

  bringup_dir = get_package_share_directory('navigation')

  return  LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(bringup_dir, 'config', 'rviz.rviz')]
        )
  ])
