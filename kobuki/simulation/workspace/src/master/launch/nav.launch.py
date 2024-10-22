import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory('master')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/odom', '/master/odom')]

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    use_namespace = LaunchConfiguration('use_namespace', default=True)

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                    #    'recoveries_server',
                       'behavior_server',
                       'bt_navigator']
    
    master_namespace = '/master'

    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(                      
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',)

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='true',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            SetParameter('use_namespace', use_namespace),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', '/warper_cmd_vel')],
                namespace=master_namespace,
                ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings,
                namespace=master_namespace,
                ),
            # Node(
            #     package='nav2_recoveries',
            #     executable='recoveries_server',
            #     name='recoveries_server',
            #     output='screen',
            #     parameters=[configured_params],
            #     remappings=remappings + [('cmd_vel', '/warper_cmd_vel')],
            #     namespace=master_namespace),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', '/warper_cmd_vel')],
                namespace=master_namespace,
                ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', '/warper_cmd_vel')],
                namespace=master_namespace,
                ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': autostart},
                            {'node_names': lifecycle_nodes}],
                namespace=master_namespace,
                ),
        ]
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(nodes)

    return ld
