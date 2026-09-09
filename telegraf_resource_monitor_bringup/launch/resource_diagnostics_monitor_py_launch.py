from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from resource_diagnostics_utils.default_paths import DEFAULT_TELEGRAF_CONFIG_PATH
from resource_diagnostics_utils.launch_arguments import declare_config_file_path, declare_log_level


def generate_launch_description():
    resource_diagnostics_updater_py_launch_dir = PathJoinSubstitution(
        [FindPackageShare('resource_diagnostics_updater_py'), 'launch']
    )

    telegraf_resource_monitor_py_launch_dir = PathJoinSubstitution(
        [FindPackageShare('telegraf_resource_monitor_py'), 'launch']
    )

    return LaunchDescription(
        [
            declare_log_level(),
            declare_config_file_path(),
            DeclareLaunchArgument(
                name='telegraf_config_path',
                default_value=DEFAULT_TELEGRAF_CONFIG_PATH,
                description='Path to the telegraf config file telegraf is started with.',
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        resource_diagnostics_updater_py_launch_dir,
                        'resource_diagnostics_updater_launch.py',
                    ]
                ),
                launch_arguments={
                    'config_file_path': LaunchConfiguration('config_file_path'),
                    'log_level': LaunchConfiguration('log_level'),
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        telegraf_resource_monitor_py_launch_dir,
                        'telegraf_resource_monitor_launch.py',
                    ]
                ),
                launch_arguments={
                    'config_file_path': LaunchConfiguration('config_file_path'),
                    'log_level': LaunchConfiguration('log_level'),
                    'telegraf_config_path': LaunchConfiguration('telegraf_config_path'),
                }.items(),
            ),
        ]
    )
