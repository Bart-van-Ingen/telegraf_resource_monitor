import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch loads this file as a module, so its own directory is not on the path yet
sys.path.insert(0, str(Path(__file__).parent))

from telegraf_launch_utils import telegraf_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config_file_path",
                # since default value is not a path, launching only this launch file will output a
                # warning specifying this fact.
                default_value="config file path not specified in launch file!",
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
                package="telegraf_resource_monitor_cpp",
                executable="telegraf_resource_monitor_node",
                name="telegraf_resource_monitor_node",
                output={"both": {"screen", "log", "own_log"}},
                emulate_tty=True,
                ros_arguments=[
                    "--log-level",
                    ["telegraf_resource_monitor_node:=", LaunchConfiguration("log_level")],
                ],
                parameters=[LaunchConfiguration("config_file_path")],
            ),
            # telegraf arguments, and telegraf itself once the node created its socket
            *telegraf_actions(),
        ]
    )
