import typing

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from telegraf_resource_monitor.sensor_message import SensorMessage


class DiagnosticsPublisherManager:
    """Manages the shared diagnostics publisher and accumulated diagnostic statuses."""

    def __init__(self, node: Node) -> None:
        self.node = node

        self.diagnostics_config = {"cpu": (("cpu", "cpu-total"),)}
        self.diagnostic_statuses: list[DiagnosticStatus] = []

        self.publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 1)

    def add_diagnostic_status(self, diagnostic_status: DiagnosticStatus) -> None:
        """Add a diagnostic status to the managed collection."""
        self.diagnostic_statuses.append(diagnostic_status)

    def publish_all_diagnostics(self) -> None:
        """Publish all accumulated diagnostic statuses."""
        self.publish_diagnostics(self.diagnostic_statuses)

    def publish_diagnostics(
        self,
        diagnostic_statuses: typing.Union[list[DiagnosticStatus], DiagnosticStatus],
    ) -> None:
        """Publish specific diagnostic statuses."""
        if isinstance(diagnostic_statuses, DiagnosticStatus):
            diagnostic_statuses = [diagnostic_statuses]

        diagnostics_array = DiagnosticArray()
        diagnostics_array.header.stamp = self.node.get_clock().now().to_msg()
        diagnostics_array.status = diagnostic_statuses

        self.node.get_logger().debug(
            f"Publishing diagnostics: {[status.name for status in diagnostic_statuses]}"
        )

        self.publisher.publish(diagnostics_array)

    def configure_resource_diagnostics(self, message, sensor_type, tags_key):

        if self.diagnostics_config.get(sensor_type, {}) == tags_key:
            self.node.get_logger().info(
                f"Enabling diagnostics for resource {sensor_type} with tags {tags_key}"
            )
            return ResourceDiagnosticsUpdater(message, self)

        return None


class ResourceDiagnosticsUpdater:
    """Creates and manages diagnostic status for a specific resource."""

    def __init__(
        self,
        sensor_message: SensorMessage,
        publisher_manager: DiagnosticsPublisherManager,
    ) -> None:

        self.publisher_manager = publisher_manager

        # Create diagnostic status for this resource
        self.diagnostic_status = DiagnosticStatus()
        self.diagnostic_status.name = sensor_message.name
        self.diagnostic_status.message = (
            f"Resource monitoring for {sensor_message.name}"
        )
        self.diagnostic_status.hardware_id = "telegraf_resource_monitor"
        self.diagnostic_status.values = [
            KeyValue(key=str(k), value=str(v)) for k, v in sensor_message.fields.items()
        ]
        self.diagnostic_status.level = DiagnosticStatus.OK

        # Register this diagnostic status with the manager
        self.publisher_manager.add_diagnostic_status(self.diagnostic_status)
        self.publish()

    def update_status(self, sensor_message: SensorMessage) -> None:
        """Update the diagnostic status for this resource."""
        self.diagnostic_status.values = [
            KeyValue(key=str(k), value=str(v)) for k, v in sensor_message.fields.items()
        ]
        self.publish()

    def publish(self) -> None:
        """Publish this resource's diagnostic status."""
        self.publisher_manager.publish_diagnostics(self.diagnostic_status)
