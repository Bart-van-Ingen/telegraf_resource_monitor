from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from resource_diagnostics_utils.launch_arguments import declare_config_file_path, declare_log_level
from resource_diagnostics_utils.telegraf_launch import telegraf_actions


def generate_launch_description():
    return LaunchDescription(
        [
            declare_log_level(),
            declare_config_file_path(),
            Node(
                package='telegraf_resource_monitor_cpp',
                executable='telegraf_resource_monitor_node',
                name='telegraf_resource_monitor_node',
                output={'both': {'screen', 'log', 'own_log'}},
                emulate_tty=True,
                ros_arguments=[
                    '--log-level',
                    ['telegraf_resource_monitor_node:=', LaunchConfiguration('log_level')],
                ],
                parameters=[LaunchConfiguration('config_file_path')],
            ),
            # telegraf arguments, and telegraf itself once the node created its socket
            *telegraf_actions(),
        ]
    )
