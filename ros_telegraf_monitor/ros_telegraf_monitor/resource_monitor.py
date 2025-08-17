from dataclasses import dataclass
import json
from pathlib import Path
from rclpy.node import Node
import socket
from rclpy.impl.rcutils_logger import RcutilsLogger
from threading import Thread
from queue import Queue
from std_msgs.msg import String
from collections import defaultdict


@dataclass
class SensorMessage:
    name: str
    tags: dict
    fields: dict
    timestamp: int

    @staticmethod
    def from_json_str(json_str: str) -> "SensorMessage":
        sensor_dict = json.loads(json_str)
        return SensorMessage(**sensor_dict)


@dataclass
class SensorMessageBuffer:
    buffer = Queue()
    logger: RcutilsLogger

    def add_message(self, message: str) -> None:
        self.logger.info(f"adding message to buffer: {message}")
        sensor_message = SensorMessage.from_json_str(message)
        self.buffer.put(sensor_message)

    def get_message(self) -> SensorMessage:
        self.logger.debug("getting message in the buffer...")
        return self.buffer.get()

    def is_empty(self) -> bool:
        return self.buffer.empty()


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
            self.logger.info(f"connected by {addr}")

            while True:
                received_data = conn.recv(1024)
                if not received_data:
                    break

                decoded_message = received_data.decode("utf-8")
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

    def __del__(self):
        self.server_socket.close()
        self.logger.info("unix socket closed.")

        # Remove existing socket file if it exists
        if self.socket_path.exists():
            self.socket_path.unlink()

        self.listener_thread.join()


class SensorMessagePublisher:
    def __init__(self, node: Node, message: SensorMessage) -> None:
        self.node = node
        self.logger = node.get_logger()

        sensor_type = message.name
        sensor_tags = message.tags

        # Convert tags dict to a pathed string
        tags_str = (
            "/".join(f"{k}/{v}" for k, v in sensor_tags.items()) if sensor_tags else ""
        )

        topic_name = f"{sensor_type}/{tags_str}"

        # sanitize topic name to avoid issues with special characters
        topic_name = topic_name.replace(" ", "_").replace("-", "_")
        # remove trailing slashes
        topic_name = topic_name.rstrip("/")

        self.logger.info(f"creating publisher for topic: {topic_name}")

        self.publisher = node.create_publisher(
            msg_type=String,
            topic=topic_name,
            qos_profile=10,
        )


class SensorMessageProcessor:
    def __init__(self, node: Node, sensor_message_buffer: SensorMessageBuffer) -> None:
        self.node = node
        self.sensor_message_buffer = sensor_message_buffer
        self.logger = node.get_logger()

        # Use defaultdict to automatically create nested dicts
        self.sensor_publishers = defaultdict(dict)

        # Bind and listen in a separate thread
        self.publisher_thread = Thread(target=self.process_buffered_messages)
        self.logger.info("starting sensor publisher thread...")
        self.publisher_thread.start()

    def __del__(self):
        self.logger.info("SensorMessagePublisher is being deleted.")

    def process_buffered_messages(self) -> None:
        while True:
            if self.sensor_message_buffer.is_empty():
                self.logger.info(
                    "no messages in buffer, waiting...", throttle_duration_sec=1.0
                )
                continue

            message: SensorMessage = self.sensor_message_buffer.get_message()

            sensor_message_publisher = self.get_publisher(message)

            # if self.sensor_publishers.get(message.name).get(message.tags) is None:

    def get_publisher(self, message: SensorMessage) -> SensorMessagePublisher:
        sensor_type = message.name
        sensor_tags = message.tags

        # Convert tags dict to a hashable key
        tags_key = tuple(sorted(sensor_tags.items())) if sensor_tags else None

        # defaultdict automatically creates the inner dict if it doesn't exist
        if tags_key not in self.sensor_publishers[sensor_type]:
            self.sensor_publishers[sensor_type][tags_key] = SensorMessagePublisher(
                self.node, message
            )

        return self.sensor_publishers[sensor_type][tags_key]


class ResourceMonitor:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.logger = node.get_logger()

        self.sensor_message_buffer = SensorMessageBuffer(self.logger)

        self.unix_socket_manager = UnixSocketManager(
            self.logger, self.sensor_message_buffer
        )

        self.sensor_message_publisher = SensorMessageProcessor(
            node, self.sensor_message_buffer
        )

        node.create_timer(
            1.0, lambda: self.logger.info("system monitoring node is running...")
        )
