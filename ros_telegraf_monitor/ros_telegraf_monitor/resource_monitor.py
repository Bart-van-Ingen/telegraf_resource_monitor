from pathlib import Path
from rclpy.node import Node
import socket
from rclpy.impl.rcutils_logger import RcutilsLogger


class UnixSocketManager:
    def __init__(self, logger: RcutilsLogger) -> None:
        self.logger = logger

        self.server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        socket_path = Path("/tmp/telegraf.sock")

        # Remove existing socket file if it exists
        if socket_path.exists():
            socket_path.unlink()

        # Bind and listen
        self.server_socket.bind(str(socket_path))
        self.server_socket.listen(1)

        logger.info(f"Server listening on {socket_path}")
    
    # destructor to close the socket
    def __del__(self):
        self.server_socket.close()
        self.logger.info("Unix socket closed.")


class ResourceMonitor:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.logger = node.get_logger()

        self.unix_socket_manager = UnixSocketManager(self.logger)

        node.create_timer(
            1.0, lambda: self.logger.info("System monitoring node is running...")
        )
