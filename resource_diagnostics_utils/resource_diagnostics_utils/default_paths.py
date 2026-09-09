from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


# must match the socket_path node parameter and outputs.socket_writer in the telegraf config
DEFAULT_SOCKET_PATH = '/tmp/telegraf.sock'

# the single telegraf config for both implementations lives in the python package
DEFAULT_TELEGRAF_CONFIG_PATH = PathJoinSubstitution(
    [FindPackageShare('resource_diagnostics_utils'), 'config', 'telegraf.conf']
)

# the single telegraf config for both implementations lives in the python package
DEFAULT_DIAGNOSTIC_CONFIG_PATH = PathJoinSubstitution(
    [FindPackageShare('resource_diagnostics_utils'), 'config', 'resource_diagnostics.yaml']
)

# resolved directly instead of through PATH, so this also works without a sourced setup.bash
TELEGRAF_BIN = PathJoinSubstitution([FindPackagePrefix('telegraf_vendor'), 'bin', 'telegraf'])
