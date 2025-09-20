import threading
from collections import defaultdict
from dataclasses import dataclass
from threading import Thread

from rclpy.node import Node

from telegraf_resource_monitor.resource_diagnostics_updater import (
    DiagnosticsPublisherManager,
    ResourceDiagnosticsUpdater,
)
from telegraf_resource_monitor.resource_publisher import ResourcePublisher
from telegraf_resource_monitor.sensor_message import SensorMessage, SensorMessageBuffer


@dataclass
class ResourceHandlers:
    publisher: ResourcePublisher
    diagnostics_updater: ResourceDiagnosticsUpdater | None = None


class SensorMessageProcessor:

    def __init__(self, node: Node, sensor_message_buffer: SensorMessageBuffer) -> None:
        self.node = node
        self.sensor_message_buffer = sensor_message_buffer

        self.logger = node.get_logger()

        self.all_resource_handlers: defaultdict[
            str, dict[tuple | None, ResourceHandlers]
        ] = defaultdict(dict)

        # Create the shared diagnostics publisher manager
        self.diagnostics_publisher_manager = DiagnosticsPublisherManager(node)

        # Add shutdown flag so that we can shutdown in unit testing
        self.shutdown_event = threading.Event()

        self.publisher_thread = Thread(target=self.process_buffered_messages)
        self.logger.debug("starting sensor publisher thread")
        self.publisher_thread.start()

    def shutdown(self):
        self.shutdown_event.set()

        # Only join if we're not calling from the same thread
        current_thread = threading.current_thread()
        if self.publisher_thread.is_alive() and self.publisher_thread != current_thread:
            self.publisher_thread.join(timeout=1.0)  # Wait max 1 second

    def process_buffered_messages(self) -> None:
        while not self.shutdown_event.is_set():
            self.sensor_message_buffer.event.wait(0.1)  # timeout to check for shutdown_event

            while not self.sensor_message_buffer.is_empty():
                message: SensorMessage = self.sensor_message_buffer.get_message()
                self.logger.debug(
                    f"publishing sensor message: {message.name} with tags: {message.tags}"
                )
                self.process_message(message)

            self.sensor_message_buffer.event.clear()

    def process_message(self, message: SensorMessage):

        resource_handlers = self.get_resource_handlers(message)

        resource_handlers.publisher.publish_from_sensor_message(message)

        if resource_handlers.diagnostics_updater:
            resource_handlers.diagnostics_updater.update_status(message)

    def get_resource_handlers(self, message: SensorMessage) -> ResourceHandlers:
        sensor_type = message.name
        sensor_tags = message.tags

        # Convert tags dict to a hashable key
        tags_key = tuple(sorted(sensor_tags.items())) if sensor_tags else None

        self.logger.debug(
            f"Getting resource handlers for sensor_type: {sensor_type}, tags_key: {tags_key}"
        )

        if tags_key not in self.all_resource_handlers[sensor_type]:
            resource_publisher = ResourcePublisher(self.node, message)
            resource_handlers = ResourceHandlers(publisher=resource_publisher)

            resource_handlers.diagnostics_updater = (
                self.diagnostics_publisher_manager.configure_resource_diagnostics(
                    message,
                    sensor_type,
                    tags_key,
                )
            )

            # defaultdict automatically creates the inner dict if it doesn't exist
            self.all_resource_handlers[sensor_type][tags_key] = resource_handlers

        return self.all_resource_handlers[sensor_type][tags_key]
