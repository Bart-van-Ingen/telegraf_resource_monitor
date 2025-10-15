from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config_file_path",
                # since default value is not a path, launching only this launch file will output a
                # warning specifying this fact.
                default_value="src/resource_diagnostics_updater/config/resource_diagnostics.yaml",
                description="Path to client specific yaml config file.",
            ),
            DeclareLaunchArgument(
                name="log_level",
                # since default value is not a path, launching only this launch file will output a
                # warning specifying this fact.
                default_value="INFO",
                description="log level of node.",
            ),
            Node(
                package="resource_diagnostics_updater",
                executable="resource_diagnostics_updater_node",
                name="resource_diagnostics_updater_node",
                output={"both": {"screen", "log", "own_log"}},
                emulate_tty=True,
                ros_arguments=[
                    "--log-level",
                    ["resource_diagnostics_updater_node:=", LaunchConfiguration("log_level")],
                ],
                parameters=[LaunchConfiguration("config_file_path")],
            ),
        ]
    )
