from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from resource_diagnostics_utils.launch_arguments import declare_config_file_path, declare_log_level
from resource_diagnostics_utils.monitor_launch_actions import telegraf_actions


def generate_launch_description():
    return LaunchDescription(
        [
            declare_log_level(),
            declare_config_file_path(),
            Node(
                package='resource_monitor_cpp',
                executable='resource_monitor_node',
                name='resource_monitor_node',
                output={'both': {'screen', 'log', 'own_log'}},
                emulate_tty=True,
                ros_arguments=[
                    '--log-level',
                    ['resource_monitor_node:=', LaunchConfiguration('log_level')],
                ],
                parameters=[LaunchConfiguration('config_file_path')],
            ),
            # telegraf arguments, and telegraf itself once the node created its socket
            *telegraf_actions(),
        ]
    )
