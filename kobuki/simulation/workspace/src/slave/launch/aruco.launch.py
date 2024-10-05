from launch import LaunchDescription
from launch_ros.actions import Node


aruco_0_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='aruco_0_tf',
    arguments=[
        '--x', '0.000',
        '--y', '0.000',
        '--z', '0.252',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_footprint_slave',
        '--child-frame-id', 'aruco_0_cell_33',
    ]
)

aruco_1_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='aruco_1_tf',
    arguments=[
        '--x', '0.000',
        '--y', '0.000',
        '--z', '0.252',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_footprint_slave',
        '--child-frame-id', 'aruco_1_cell_33',
    ]
)

aruco_2_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='aruco_2_tf',
    arguments=[
        '--x', '0.000',
        '--y', '0.000',
        '--z', '0.252',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_footprint_slave',
        '--child-frame-id', 'aruco_2_cell_33',
    ]
)

aruco_3_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='aruco_3_tf',
    arguments=[
        '--x', '0.000',
        '--y', '0.000',
        '--z', '0.252',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '0.0',
        '--frame-id', 'base_footprint_slave',
        '--child-frame-id', 'aruco_3_cell_33',
    ]
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(aruco_0_tf)
    ld.add_action(aruco_1_tf)
    ld.add_action(aruco_2_tf)
    ld.add_action(aruco_3_tf)
    
    return ld
