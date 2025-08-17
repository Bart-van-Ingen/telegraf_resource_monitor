from collections import defaultdict
from rclpy.node import Node
from threading import Thread

from telegraf_resource_monitor.sensor_message import SensorMessage, SensorMessageBuffer
from telegraf_resource_monitor.sensor_message_publisher import SensorMessagePublisher


class SensorMessageProcessor:
    def __init__(self, node: Node, sensor_message_buffer: SensorMessageBuffer) -> None:
        self.node = node
        self.sensor_message_buffer = sensor_message_buffer
        self.logger = node.get_logger()

        # Use defaultdict to automatically create nested dicts
        self.sensor_publishers = defaultdict(dict)

        # Bind and listen in a separate thread
        self.publisher_thread = Thread(target=self.process_buffered_messages)
        self.logger.debug("starting sensor publisher thread")
        self.publisher_thread.start()

    def __del__(self):
        self.publisher_thread.join()

    def process_buffered_messages(self) -> None:
        while True:
            # Wait for the event to be set (indicating new messages are available)
            self.sensor_message_buffer.event.wait()

            # Process all available messages
            while not self.sensor_message_buffer.is_empty():
                message: SensorMessage = self.sensor_message_buffer.get_message()
                self.publish_sensor_message(message)

            # Clear the event after processing all messages
            self.sensor_message_buffer.event.clear()

    def publish_sensor_message(self, message: SensorMessage) -> None:
        self.logger.debug(
            f"publishing sensor message: {message.name} with tags: {message.tags}"
        )

        sensor_message_publisher = self.get_publisher(message)
        sensor_message_publisher.publish(message)

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
