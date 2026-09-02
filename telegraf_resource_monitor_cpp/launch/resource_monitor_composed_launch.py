import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ros2 launch loads this file as a module, so its own directory is not on the path yet
sys.path.insert(0, str(Path(__file__).parent))

from telegraf_launch_utils import telegraf_actions


def generate_launch_description():
    """Run the telegraf monitor and the diagnostics updater together in one process.

    Both nodes are loaded as components into a single container. The config file is
    passed to both, each node picks up its own section by node name.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='config_file_path',
                # since default value is not a path, launching only this launch file will output a
                # warning specifying this fact.
                default_value='config file path not specified in launch file!',
                description='Path to client specific yaml config file.',
            ),
            DeclareLaunchArgument(
                name='log_level',
                default_value='INFO',
                description='log level of the nodes in the container.',
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
            # telegraf arguments, and telegraf itself once the node created its socket
            *telegraf_actions(),
        ]
    )
