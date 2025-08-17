from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration


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
            Node(
                package="telegraf_resource_monitor",
                executable="telegraf_resource_monitor",
                name="telegraf_resource_monitor",
                output={"both": {"screen", "log", "own_log"}},
                emulate_tty=True,
                ros_arguments=["--log-level", "telegraf_resource_monitor:=DEBUG"],
                parameters=[LaunchConfiguration("config_file_path")],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "telegraf",
                            "--config",
                            "/ros_ws/src/telegraf_resource_monitor/config/telegraf.conf",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
