import asyncio
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueCoroutine,
    RegisterEventHandler,
)
from launch.event_handlers import OnExecutionComplete
from launch.substitutions import LaunchConfiguration

# the single telegraf config for both implementations lives in the python package
DEFAULT_TELEGRAF_CONFIG_PATH = str(
    Path(get_package_share_directory('telegraf_resource_monitor_py')) / 'config' / 'telegraf.conf'
)

# resolved directly instead of through PATH, so this also works without a sourced setup.bash
TELEGRAF_BIN = str(Path(get_package_prefix('telegraf_vendor')) / 'bin' / 'telegraf')

# must match the socket_path node parameter and outputs.socket_writer in the telegraf config
DEFAULT_SOCKET_PATH = '/tmp/telegraf.sock'

SOCKET_WAIT_PERIOD = 0.05
SOCKET_WAIT_TIMEOUT = 10.0


# a coroutine, because sleeping in a launch action blocks the loop that still starts the node
async def wait_for_socket(context):
    socket_path = Path(LaunchConfiguration('socket_path').perform(context))

    # a killed run leaves its socket file behind and telegraf exits on a socket nobody listens
    # on. the node unlinks the file as well before it binds its own socket.
    socket_path.unlink(missing_ok=True)

    deadline = time.monotonic() + SOCKET_WAIT_TIMEOUT
    while not socket_path.is_socket():
        if time.monotonic() > deadline:
            raise RuntimeError(
                f'{socket_path} was not created within {SOCKET_WAIT_TIMEOUT} seconds, '
                'not starting telegraf'
            )
        await asyncio.sleep(SOCKET_WAIT_PERIOD)


# launch completes a coroutine whether it succeeded or not, hence the second check
def start_telegraf(event, context):
    return ExecuteProcess(
        cmd=[TELEGRAF_BIN, '--config', LaunchConfiguration('telegraf_config_path')],
        output='screen',
    )


# telegraf exits when it cannot connect, so it only starts once the node created the socket
def telegraf_actions():
    wait_for_socket_action = OpaqueCoroutine(coroutine=wait_for_socket)

    return [
        DeclareLaunchArgument(
            name='telegraf_config_path',
            default_value=DEFAULT_TELEGRAF_CONFIG_PATH,
            description='Path to the telegraf config file telegraf is started with.',
        ),
        DeclareLaunchArgument(
            name='socket_path',
            default_value=DEFAULT_SOCKET_PATH,
            description=(
                'Path of the unix socket the node creates. Only used to wait for that socket '
                'before telegraf starts, so it must match the socket_path node parameter and '
                'the outputs.socket_writer address in the telegraf config.'
            ),
        ),
        # registering this after the wait action would miss its completion event
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=wait_for_socket_action,
                on_completion=start_telegraf,
            )
        ),
        wait_for_socket_action,
    ]
