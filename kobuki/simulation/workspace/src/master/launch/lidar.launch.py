from launch import LaunchDescription
from launch_ros.actions import Node


lidar_transform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_static_transform_publisher',
    arguments=[
        '--x', '0.000',
        '--y', '0.000',
        '--z', '0.252',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '3.14',
        '--frame-id', 'base_footprint',
        '--child-frame-id', 'lidar',
    ]
)


lidar_node = Node(
    package='rplidar_ros',
    executable='rplidar_composition',
    name='rplidar_composition',
    parameters=[
        {'serial_port': '/dev/ttyUSB0'},
        {'frame_id': 'lidar'},
        {'angle_compensate': True},
        {'scan_mode': 'Standard'},
        {'serial_baudrate': 115200},
        {'scan_frequency': 10.0},
        {'sample_rate': 1},
    ]
)


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(lidar_transform)
    ld.add_action(lidar_node)
    
    return ld
