from launch.actions import DeclareLaunchArgument

from resource_diagnostics_utils.default_paths import (
    DEFAULT_DIAGNOSTIC_CONFIG_PATH,
    DEFAULT_TELEGRAF_CONFIG_PATH,
)


def declare_log_level():
    return DeclareLaunchArgument(
        name='log_level',
        default_value='INFO',
        description='log level of the nodes started by this launch file.',
    )


def declare_config_file_path():
    return DeclareLaunchArgument(
        name='config_file_path',
        default_value=DEFAULT_DIAGNOSTIC_CONFIG_PATH,
        description='Path to client specific yaml config file.',
    )


def declare_telegraf_config_path():
    return DeclareLaunchArgument(
        name='telegraf_config_path',
        default_value=DEFAULT_TELEGRAF_CONFIG_PATH,
        description='Path to the telegraf config file telegraf is started with.',
    )
