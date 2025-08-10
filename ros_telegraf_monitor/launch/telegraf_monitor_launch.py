from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name='config_file_path',
                # since default value is not a path, launching only this launch file will output a
                # warning specifying this fact.
                default_value='config file path not specified in launch file!',
                description='Path to client specific yaml config file.',
            ),

            Node(
                package='ros_telegraf_monitor',
                executable='telgraf_monitor_node',
                name='telgraf_monitor_node',
                output={'both': {'screen', 'log', 'own_log'}},
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('config_file_path')
                ]
            ),
            ExecuteProcess(
                cmd=['telegraf', '--config', 'src/ros_telegraf_monitor/config/telegraf.conf'],
                    output='screen')
        ]
    )
