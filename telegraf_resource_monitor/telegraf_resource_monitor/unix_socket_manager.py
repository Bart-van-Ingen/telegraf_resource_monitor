import socket
from pathlib import Path
from threading import Thread

from rclpy.impl.rcutils_logger import RcutilsLogger

from telegraf_resource_monitor.sensor_message import SensorMessageBuffer


class UnixSocketManager:
    def __init__(
        self, logger: RcutilsLogger, sensor_message_buffer: SensorMessageBuffer
    ) -> None:
        self.logger = logger
        self.sensor_message_buffer = sensor_message_buffer

        self.server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket_path = Path("/tmp/telegraf.sock")
        logger.info(f"server listening on {self.socket_path}")

        # Remove existing socket file if it exists
        if self.socket_path.exists():
            self.socket_path.unlink()

        # Bind and listen in a separate thread
        self.listener_thread = Thread(target=self.start_socket_listener)
        logger.info("starting unix socket listener thread...")
        self.listener_thread.start()

    def start_socket_listener(self):
        self.server_socket.bind(str(self.socket_path))
        self.server_socket.listen()
        conn, addr = self.server_socket.accept()

        message_buffer = ""

        with conn:
            self.logger.debug(f"connected by {addr}")

            while True:
                received_data = conn.recv(1024)
                if not received_data:
                    break

                decoded_message = received_data.decode("utf-8")
                message_buffer = self.parse_complete_messages(
                    decoded_message, message_buffer
                )

    def parse_complete_messages(self, decoded_message, message_buffer):

        message_lines = decoded_message.split("\n")

        # Process all complete entries (all but the last split part)
        if len(message_lines) > 1:
            # Add the first part to our current entry and process it
            message_buffer += message_lines[0]
            self.sensor_message_buffer.add_message(message_buffer)
            message_buffer = ""

            # Process any additional complete entries
            for complete_line in message_lines[1:-1]:
                self.sensor_message_buffer.add_message(complete_line)

                # Keep the last part (which may be incomplete) for the next iteration
        message_buffer += message_lines[-1]

        return message_buffer

    def __del__(self):
        self.server_socket.close()
        self.logger.info("unix socket closed.")

        # Remove existing socket file if it exists
        if self.socket_path.exists():
            self.socket_path.unlink()

        self.listener_thread.join()
