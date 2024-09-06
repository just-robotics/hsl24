#!/usr/bin/env python3

# Written by Nikolay Dema <ndema2301@gmail.com>, September 2022

import os

from launch import LaunchDescription

from launch.actions import ExecuteProcess

from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = "/workspace/src/survey"

    os.environ["GAZEBO_MODEL_PATH"] = pkg_path + "/models"

    world_path = pkg_path + "/worlds/ozyland.world"

    return LaunchDescription([

        ExecuteProcess(output = "screen",
                       cmd    = ["gazebo",
                                 "--verbose",
                                 "-s", "libgazebo_ros_init.so",
                                 world_path]),

        Node(package    = "tf2_ros",
             executable = "static_transform_publisher",
             arguments  = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
             output     = "screen"),

        Node(package    = "tf2_ros",
             executable = "static_transform_publisher",
             arguments  = ["0", "0", "0.775", "0", "0", "0", "base_link", "lidar"],
             output     = "screen"),

        Node(package    = "tf2_ros",
             executable = "static_transform_publisher",
             arguments  = ["0.42", "0", "1.75", "-0.5", "0.5", "-0.5", "0.5", "base_link", "camera"],
             output     = "screen")

    ])
