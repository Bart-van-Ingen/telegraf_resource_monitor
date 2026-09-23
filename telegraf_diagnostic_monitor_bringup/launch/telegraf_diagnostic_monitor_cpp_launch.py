from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from resource_diagnostics_utils.launch_arguments import declare_config_file_path, declare_log_level
from resource_diagnostics_utils.monitor_launch_actions import collectd_actions, telegraf_actions


def generate_launch_description():

    return LaunchDescription(
        [
            declare_log_level(),
            declare_config_file_path(),
            DeclareLaunchArgument(
                name='monitor_type',
                default_value='telegraf',
                choices=['telegraf', 'collectd'],
                description='the type of monitor to launch, options: telegraf, collectd',
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
            GroupAction(
                actions=telegraf_actions(),
                scoped=False,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('monitor_type'), 'telegraf')
                ),
            ),
            GroupAction(
                actions=collectd_actions(),
                scoped=False,
                condition=IfCondition(
                    EqualsSubstitution(LaunchConfiguration('monitor_type'), 'collectd')
                ),
            ),
        ]
    )
