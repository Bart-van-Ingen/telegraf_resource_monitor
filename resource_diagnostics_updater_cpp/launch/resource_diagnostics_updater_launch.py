from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from resource_diagnostics_utils.launch_arguments import declare_config_file_path, declare_log_level


def generate_launch_description():
    return LaunchDescription(
        [
            declare_log_level(),
            declare_config_file_path(),
            Node(
                package='resource_diagnostics_updater_cpp',
                executable='resource_diagnostics_updater_node',
                name='resource_diagnostics_updater_node',
                output={'both': {'screen', 'log', 'own_log'}},
                emulate_tty=True,
                ros_arguments=[
                    '--log-level',
                    ['resource_diagnostics_updater_node:=', LaunchConfiguration('log_level')],
                ],
                parameters=[LaunchConfiguration('config_file_path')],
            ),
        ]
    )
