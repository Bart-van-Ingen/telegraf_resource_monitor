from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from resource_diagnostics_utils.default_paths import DEFAULT_TELEGRAF_CONFIG_PATH
from resource_diagnostics_utils.launch_arguments import declare_config_file_path, declare_log_level
from resource_diagnostics_utils.telegraf_launch import telegraf_actions


def generate_launch_description():

    return LaunchDescription(
        [
            declare_log_level(),
            declare_config_file_path(),
            DeclareLaunchArgument(
                name='telegraf_config_path',
                default_value=DEFAULT_TELEGRAF_CONFIG_PATH,
                description='Path to the telegraf config file telegraf is started with.',
            ),
            ComposableNodeContainer(
                name='resource_monitor_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                output={'both': {'screen', 'log', 'own_log'}},
                emulate_tty=True,
                ros_arguments=[
                    '--log-level',
                    ['telegraf_resource_monitoring_node:=', LaunchConfiguration('log_level')],
                    '--log-level',
                    ['resource_diagnostics_updater_node:=', LaunchConfiguration('log_level')],
                ],
                composable_node_descriptions=[
                    ComposableNode(
                        package='telegraf_resource_monitor_cpp',
                        plugin='TelegrafResourceMonitorNode',
                        name='telegraf_resource_monitoring_node',
                        parameters=[LaunchConfiguration('config_file_path')],
                        # both nodes need this, otherwise the messages go through DDS.
                        # see docs/intra_process_communication.md
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                    ComposableNode(
                        package='resource_diagnostics_updater_cpp',
                        plugin='ResourceDiagnosticsUpdaterNode',
                        name='resource_diagnostics_updater_node',
                        parameters=[LaunchConfiguration('config_file_path')],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ],
            ),
            *telegraf_actions(),
        ]
    )
